# ADR 0001: Plan-only is the default

## Status

Accepted.

## Decision

Every demonstration starts in plan-only mode. A request must explicitly set `execute=true`, and the motion node must also permit execution through configuration before any trajectory reaches a controller.

## Rationale

This makes the safer state the default, keeps planning demonstrations useful without hardware, and prevents a copied launch command from silently commanding an attached arm. It is an application guard, not a certified safety function.

