# Launch inhibition retention and managed metadata

A completed Launch inhibition is a monotonic local fact until explicit verified recovery. Later managed/network flight-state metadata in states7–14 must not repeat a backup-register write or create a new Abort merely because later backup access fails. Current owner completion or a full verified marker restored at warm initialization supplies this metadata-only fact. Corruption/loss after that observation does not turn a metadata update into another physical Launch admission.

The metadata path keeps its existing sender, original-time floor, current Stop, recovery, and no-regression checks. Managed Aborted and prelaunch regression do not become manual Stop authority or clear inhibition. Metadata never queues a sequence, fires an igniter or grants new physical execution authority.

Initial and genuinely new physical Launch admission still independently verifies the backup write and retains the existing local safe-off/refusal behavior on failure. A completed-marker shortcut must not replace that physical check.

STM32 backup registers survive warm NRST/NVIC/IWDG resets while their backup domain retains power. They do not prove persistence through loss of backup-domain power. A full valid retained pair may restore the local metadata fact on warm boot; zero or torn pairs cannot borrow that verified fact. Existing boot-generation/original-time recovery protections still apply. New intent-bound distributed Launch tokens remain separate, unimplemented work; no cold old-legacy-command or hardware guarantee is inferred from this metadata change.
