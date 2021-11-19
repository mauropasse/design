---
layout: default
title: Events Executor
abstract:
  This article describes the design of an events based executor as an alternative to the waitsets approach.
published: true
author: '[Mauro Passerino - iRobot](https://github.com/mauropasse) [Lenny Story - iRobot](https://github.com/Codematic71) [Alberto Soragna - iRobot](https://github.com/alsora)'
---

{:toc}

# {{ page.title }}

<div class="abstract" markdown="1">
{{ page.abstract }}
</div>

Original Author: {{ page.author }}

## Motivations

Many articles have discussed the CPU overhead of the default ROS 2 executor the `SingleThreadedExecutor` and of its improved version the `StaticSingleThreadedExecutor`.
 - [Reducing ROS 2 CPU overhead by simplifying the ROS 2 layers](https://discourse.ros.org/t/reducing-ros-2-cpu-overhead-by-simplifying-the-ros-2-layers/13808)
 - [SingleThreadedExecutor creates a high CPU overhead in ROS 2](https://discourse.ros.org/t/singlethreadedexecutor-creates-a-high-cpu-overhead-in-ros-2/10077)
 - [ROS 2 Middleware Change Proposal](https://discourse.ros.org/t/ros2-middleware-change-proposal/15863)

Some major contributors to this overhead can be identified:
 - The cost of modifying and updating waitsets.
   Currently this operation happens multiple times for every iteration of the executor, even if the majority of robotics systems are mostly static, i.e. the number of ROS 2 entities is fixed and known at startup.
   A detailed walk-through of the waitset management process can be found [here](https://discourse.ros.org/t/ros2-middleware-change-proposal/15863/6).
 - Inefficiency of the timers management.
   This is currently done in the rcl layer, where at each iteration the full list of timers associated with an executor is checked twice.
 - The presence of several abstraction layers that do not simply forward data, but rather perform non trivial operations, between the application and the underlying middleware.
 
The CPU overhead of ROS 2 with respect to other pub/sub systems (as the same DDS middlewares that it uses under the hood), makes it difficult to integrate ROS 2 on embedded platforms with constrained resources, as a not negligible amount of CPU is needed just for the executor to identify entities with work to do.
 
## Requirements
 
The following requirements have been taken account for the design of a new ROS 2 executor:
 - Good performance: the executor shall not be the CPU bottleneck of a ROS 2 system, thus allowing to obtain performance comparable to the ones of the underlying middleware that is being used.
 - Scalability: The executor must perform well independently of the system size, e.g. the latency for processing a fixed amount of messages must be independent from the number of entities.
   This can flow into the "You don't pay for what you don't use" paradigm.
 - Extensibility: The executor should set a base from which it is straight-forward to add more functionalities without impacting performances or requiring heavy modifications of the ROS2 core libraries.
 - Ordering of events: The executor should process events in the same order they have occurred.
 - High precision and accurate timers: The executor should provide mechanisms to ensure that timers are executed with the minimum possible latency.
 
## The Events Executor

The so-called `EventsExecutor` class has been designed in order to address the aforementioned requirements.
The `EventsExecutor` inherits from the base `Executor` class.
However it is inherently different from the previous executor implementations (e.g. the `SingleThreadedExecutor`, the `MultiThreadedExecutor` and the `StaticSingleThreadedExecutor`).
All these executor classes indeed rely on abstractions of the DDS waitsets in order to be notified when entities have work to do.
On the other hand, the `EventsExecutor` uses a completely separate approach, i.e. the DDS listener APIs (see for example [fast-dds documentation](https://fast-dds.docs.eprosima.com/en/latest/fastdds/dds_layer/subscriber/dataReaderListener/dataReaderListener.html)).
Note that the `EventsExecutor` is only one of the possible execution classes that could be built on top of the aforementioned APIs.

### Brief Overview

In order to execute ROS 2 entities, the `EventsExecutor` relies on an events queue.
Whenever a ROS 2 entity that is associated to the executor has work to do, it will push an event into the executor's queue.
For DDS-based entities (e.g. `rclcpp::Subscription`, `rclcpp::Service`, etc.), events will be pushed by properly set DDS listener callbacks.
Timers are handled differently, using a dedicated `TimersManager` component that will be described in details in the rest of this document.

Then the executor can extract the events from the queue and process them.
Processing an event results in different operations depending on the entity that generated it.

![Overview](../img/events_executor/overview.png)

## ExecutorEvent

The `EventsExecutor` requires that all the entities as soon as they have some work to do, they push an `ExecutorEvent` into the executor's `EventsQueue`.
Events might be produced in the RMW layer, i.e. where the underlying middleware notifies the ROS 2 entity that there is new work to do, but events can also be produced in the rclcpp layer or by the application itself.
Regardless of their origin, events are executed by the `EventsExecutor` in its context.

Being this implementation an events-queue, rather than a data-queue, the `ExecutorEvent` data structures do not contain the actual data received, but rather they contain all what is needed for the executor to be able to process that particular event.
In practice this means that `ExecutorEvent` data structures include identifiers to the rclcpp object that should process the generated event.

Considering the entities currently available in ROS 2, the content of the `ExecutorEvent` event data structure can be any of the following:
 - `ExecutorEventType::SUBSCRIPTION_EVENT` and an identifier for a `rclcpp::SubscriptionBase` object.
 - `ExecutorEventType::SERVICE_EVENT` and an identifier for a `rclcpp::ServiceBase` object.
 - `ExecutorEventType::CLIENT_EVENT` and an identifier for a `rclcpp::ClientBase` object.
 - `ExecutorEventType::TIMER_EVENT` and an identifier for a `rclcpp::TimerBase` object.
 - `ExecutorEventType::WAITABLE_EVENT` and an identifier for a `rclcpp::Waitable` object.
     
Let's consider as an example how ROS 2 subscription are handled in the current implementation of the `EventsExecutor`.
The underlying middleware will notify the rmw_subscription object whenever a new message arrives.
The rmw_subscription object will have to push an event data structure into the `EventsExecutor`'s queue.
This event data structure will contain the `ExecutorEventType::SUBSCRIPTION_EVENT` label (to denote that this event comes from a subscription) and an identifier to the `rclcpp::SubscriptionBase` object which will have to execute it.

Note that `ExecutorEvent` generated by waitables are slightly different.
Indeed waitables can potentially wrap multiple DDS entities and need to know which one generated an event in order to process it.
So, in the case of an entity which derives from `rclcpp::Waitable`, for example a `rclcpp_action::ServerBase` which has a request from a client, the rmw pushes an `ExecutorEventType::WAITABLE_EVENT` along with the ID of the `rclcpp_action::ServerBase` object, but also includes the ID of the action server's `GoalService` which should be executed when the `EventsExecutor` executes the action server waitable.

## Pushing ExecutorEvent

It is responsibility of the `EventsExecutor` to setup entities such that they can push `ExecutorEvent`s.

An application can add ROS 2 nodes or callback groups to an `EventsExecutor` using the corresponding APIs (e.g. `add_node()` and `add_callback_group()`).
Added nodes and callback groups are inspected in order to find all the existing ROS 2 entities that have to be associated with the executor.
Whenever new entities are associated to the `EventsExecutor` an initialization procedure takes place, with the purpose of provisioning those entities with a way for pushing events.
In simple terms, the initialization procedure simply consists in passing a function pointer to the entity.
The function will have no return type and it will take as input the number of events that should be pushed.

The initialization procedure presents some small variations depending on the type of the entity:

##### Client/Server/Subscription

These entities have a 1-to-1 correspondence between objects in the rclcpp and the rmw layer.
The initialization is straight forward and it entirely relies on setting the aforementioned function pointer as a DDS listener function.

##### Waitable

`Waitables` are a concept that exists only in the rclcpp layer and that is used to implement a wide variety of different entities.
Because of this, it is not possible to define a common way for pushing events.
Each class that inherits from `rclcpp::Waitable` and that wants to be used with the `EventsExecutor` will have to define its own function for doing so.
For example, a `rclcpp::SubscriptionIntraProcess`, which relies on a guard condition and it's entirely managed in the rclcpp layer, will be able to directly push events.
On the other hand, a `QOSEventHandler` will forward the aforementioned items to the underlying rmw QoSEvent implementation.

Note that `Waitables` can be used as a way to implement generic custom events, as it will be described in the next sections.

##### Timer

As mentioned before, when used in an `EventsExecutor`, timers are managed by a `TimersManager` object.
The `TimersManager` is owned by the `EventsExecutor` and it has access to the `EventsQueue`.
So, whenever a timer is ready, it can push the corresponding events there.

## EventsQueue

The `EventsQueue` class is responsible for storing events and providing them to the `EventsExecutor`.
This is a critical component and, depending on its implementation, it is possible to obtain very different execution models.
Due to the presence of very different use-cases in the robotics domain, each with their own requirements and priorities, a single `EventsQueue` class wouldn't be able to efficiently support all of them.

Because of this, the `EventsExecutor` provides only an `EventsQueue` interface class.
This is a very simple interface and it includes the standard operations that can be found in generic STL containers.
The only addition is an `init()` API which will allow the `EventsQueue` to access some internal details of the `EventsExecutor` if needed.

The `rclcpp` library will provide only few examples of simple `EventsQueue` implementations, while allowing developers to define their own classes as part of their applications.
This is achieved by having the `EventsQueue` object be created in the application and then being passed to the `EventsExecutor` constructor as an argument.

In the following sections, some simple examples of `EventsQueue` implementations will be described.
These will focus on showing what are some common storing and pruning strategies.
All these implementation use standard STL containers under the hood, but developers are invited to follow these examples and use more advanced data-structures to improve performance.

It is important to note that all the queue implementations described here lead to overall better performance than the existing executors.

##### SimpleEventsQueue

This is the simplest possible `EventsQueue` implementation and it is just a wrapper around the `std::queue` with the addition of `std::mutex` for thread safety.
It does not use any information about the entities producing the events, for the sake of simplicity.

When new events are pushed, they are immediately added to the underlying queue.
Events will be extracted only when the `EventsExecutor` takes them.

Due to its simplicity and lack of any book-keeping mechanism, this implementation is very efficient.
However, it will grow unbounded if the `EventsExecutor` is not spinning or if events are being pushed faster than they are processed.

As long as the number of events in the queue produced by each entity does not exceed the entity history QoS, this very simple queue also guarantees that events are processed in the same order as they were produced.

##### BoundedEventsQueue

This queue is an extension of the `SimpleEventsQueue` which provides access to different policies to prevent the unbounded growth.

Whenever a new `ExecutorEvent` is pushed, different conditions and recovery actions will be adopted depending on the chosen policy.
 - `NotBounded`: nothing will happen.
 The behavior of the queue is identical to the `SimpleEventsQueue` described before.
 - `BoundedNoTimeOrdering`: the event will be pushed only if the maximum number of events for this entity has not been reached.
 If an entity has a QoS history size of 10, this means avoid pushing an 11th event into the queue.
 Note that, not pushing the event does not result in losing data, but rather it only affects the order of execution.
 - `BoundedWithTimeOrdering`: an event is always pushed, but, if the maximum number of events for this entity has been reached, the oldest event for this entity will also be removed.

It is easy to notice how the different policies perform various trade-off between performance (CPU time) and correctness.

##### WaitSetEventsQueue

This queue is meant to provide the same execution model of waitset-based executors, while at the same time maintaining some of the improvements of the `EventsExecutor` approach.

Similarly to a waitset, this approach is bounded and entities are executed in a fixed and predetermined order, rather than depending on the chronological order of events.

This `EventsQueue` implementation does not rely on an abstraction of the DDS waitset, but rather it uses the same notification mechanism as other queues.
Moreover, only entities which have work to do are present in the queue, avoiding to pay performance for polling entities that are not ready.

Entity events are represented by the entity ID and a counter, specifying the number of events left to execute.
This counter is bounded by the history size (QoS depth) of the entity.

![Overview](../img/events_executor/waitset-events-queue.png)

### TimersManager

The `TimersManager` is a class that allows to monitor timers.
It should respect the following specification:
 - Timers are dynamic and they can be added or removed while the `TimersManager` is running.
 - The `TimersManager` should support both executing a timer or push a `ExecutorEventType::TIMER_EVENT` with the ID of the `rclcpp::TimerBase` when ready.
 - The `TimersManager` should support both periodic as well as one-shot timers (with the second currently not available in ROS 2).
 - The `TimersManager` need to support all the modes of a ROS 2 executor (i.e. `spin()`, `spin_some()`, etc).
 - Users should be able to extend the `TimersManager` to improve its performance according to their specific use-case.

It's important to note that the `TimersManager` can be configured to either push events into the queue when a timer is ready or rather to execute it directly from its own context.

In order to use the `TimersManager` within a blocking `spin()` call, a `TimersManager` task is started.
This task will continuously monitor timers and sleep until the next timer is ready as long as the executor is still spinning.
For example, the current implementation executes this task through the following loop:
 1. Get the time before the first timer expires.
 2. Sleep for the required amount of time.
 3. Execute or push timer events of all ready timers.
 4. Repeat.

Creating a new task and executing timers there provides the most efficient way for handling timers and it ensures that timers are executed in a timely manner, without having to wait for other entities to be processed.
However, this may not be compatible with all use-cases or with the non-blocking variants of `spin()`.
To implement these variants, the following APIs are exposed:
 - `get_head_timeout()` which returns the time before the first timer expires.
 - `execute_head_timer()` which execute the first timer if it's ready.
 - `execute_ready_timers()` which execute all the ready timers if any.

By using these APIs, it is possible to implement the non-blocking variants of `spin()` without the need of additional threads for monitoring the timers.
This has the advantage to give to the executor a more fine-grained control on which and how many timers are executed, and saves the overhead of continuously starting and killing a thread if the `spin()` variants is called within a loop.

The current implementation of the `TimersManager` uses an heap priority queue to keep the timers sorted.
Whenever a timer is added or removed from the `TimersManager`, the queue is heapified again (i.e. reordered to be a valid heap).
After a timer is executed (or an `ExecutorEventType::TIMER_EVENT` is pushed) its expire time will be updated, so it's necessary to provide an efficient operation for updating the root element of the priority queue, while the rest of it is still correctly ordered.
This is currently done using the `pop_heap()` function from std library, that has 2 log(n) complexity.

Moving the timers management into its own class allows to isolate this feature from the rest of the system, allowing to develop it independently.
Moreover, by separating timers execution from sorting, it is possible to extend the `TimersManager` object to use different algorithms -such as timer wheels- or to take advantage of the OS capabilities by using low level OS timers, without having to modify the executor.

### Events Execution

The simplest usage of the `EventsExecutor` consists in calling the `spin()` method which results in having the `EventsExecutor` to start the `TimersManager` task, while at the same time continuously waiting for events to be pushed into the `EventsQueue`.
Events are executed in a FIFO manner, following the order in which they are pushed into the `EventsQueue`.

Executing an event is done by calling the corresponding API on the rclcpp object identified by the event.
Let's consider a ROS 2 subscription as an example.
The event will contain the `ExecutorEventType::SUBSCRIPTION_EVENT` label and an identifier to a `rclcpp::SubscriptionBase` object, and the executor will use them to get the `rclcpp::SubscriptionBase` object to call its `execute_subscription()` API.

`Waitables` implement their own `execute()` API which can be called directly by the executor when a `ExecutorEventType::WAITABLE_EVENT` is received.
The callback also receives an identifier argument, needed because a Waitable may be composed of several distinct entities such as subscriptions, services, etc.
This implies that the provided callback can use the identifier to behave differently depending on which entity triggered the waitable to become ready.

The `EventsExecutor` should allow entities to push events into the queue while other events are being processed, i.e. without blocking the producers.
The current implementation achieves that by having two separate `EventsQueue`: one for storage and accumulation of events and the other for their execution, and swapping the two whenever the executor wakes up.

In order to implement the non-blocking variants of `spin()` it is possible to get from the `TimersManager` the duration until the first timer expires and then use this value as a timeout in the conditional wait for new events.

The `EventsExecutor` should take care of correctly handling events that have been generated from entities that are not associated with the executor anymore, either because they have been removed or because they went out of scope.

![Events Execution](../img/events_executor/execution.png)

The `EventsExecutor` design allows to easily extend the execution of events from a single thread to a multi thread one, simply by creating new threads whenever an entity has to be executed.

### Cleanup and ownership

The `EventsExecutor` should not keep ownership of the nodes and of the entities that are associated to it.

Whenever a node or callback group is removed from the executor using the corresponding APIs (`remove_node()` and `remove_callback_group()`), the `EventsExecutor` needs to clean up the associated entities to make them stop sending events.
Entities may be eventually added back again to the same executor or to a different one and they need to be able to start again producing events.
This requires that the initialization procedure that is performed when entities are added to an executor is completely reversible.

The events-based approach requires particular care in handling the case where entities are destroyed while still associated with an executor.
An entity may push an event into the `EventsQueue` and then get immediately destroyed.
This may happen before the `EventsExecutor` start to process those events or while it is processing them.

The current implementation addresses this problem by keeping a list of weak pointers of entities.
So before trying to execute them, a check is performed to confirm the entity hasn't expired.
If the weak pointer has expired, then it is removed from the list, otherwise the entity is executed normally.
