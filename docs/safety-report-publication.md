# Local automatic Abort report publication

The first safety diagnostic remains immutable for the boot, including its original UTC. It is separate from the current pending network report: each successfully claimed local cause carries its owner Abort epoch and original UTC, so a later genuine cause can be reported after explicit recovery without rewriting the first diagnostic.

The network task claims one finite publication lease under the state owner, releases the state mutex, and performs one SDK enqueue. It holds neither the state mutex nor an interrupt mask across SDK allocation/queueing. Only the exact lease completion can release that publisher; an old completion cannot release a newer retry for the same cause. SDK refusal retains the current cause for the existing one-second retry cadence.

Explicit recovery closes new publication and retires the old pending cause. Its actual owner completion waits asynchronously for a previously claimed publisher to return. Manual Stop remains independent and immediately invalidates the recovery epoch; it never waits for the SDK publisher. A request that expires while waiting remains refused/stopped and is not automatically replayed.

Launch admission also closes publication. A synchronous Launch/state/sequence API returns refusal while a publisher is active; it never reports success that could authorize early firing. The owner service can finish durable inhibition after the publisher drains, but does not replay Launch or actuate outputs. A subsequent explicit or reliable admission must succeed separately. Actual backup-register verification failure preserves the existing local safe Stop; merely waiting for a publisher does not create an automatic Abort report.

Old packets already retained by SDK are not recalled. Their receivers still apply original-time retirement and durable Launch inhibition. This local contract does not prove that every remote board is inhibited when GS first accepts an SDK queue item; a distributed prepare/inhibit/intent-bound-commit protocol remains separate work.

`test_safety_report_publication.py` compiles the production reporter and actual recovery, owner, persistence and Launch-latch source with a held SDK-boundary fixture in two host threads. It covers Pending recovery/Launch refusal, independent Manual Stop, original-intent expiry, a new real cause with unknown UTC0, repeated retained-state setters, and stale completion of a newer same-cause lease. It does not claim hardware actuation or radio delivery.
