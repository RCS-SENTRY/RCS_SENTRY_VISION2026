# Sentry Rule Constraints Audit

This note records the RM2026 rule constraints that affect sentry rule-action
selection in `sentry_bt`.

## Findings

- P0: `EXCHANGE_AMMO_AT_POINT` must not treat `on_fortress` as a normal
  projectile allowance exchange point.
- P1: `CLAIM_PERIODIC_AMMO` is not part of the official phase-1 sentry
  autonomous decision command fields and must not be connected from the main BT.
- P2: the `sentry_bt` simulator must follow the same ordinary exchange-point
  constraints as production rule logic.

## Official Constraint Interpretation

Ordinary projectile allowance exchange is only allowed at:

- supply area
- base buff point
- outpost buff point

Fortress reserve allowance is a separate rule concept. It must not reuse the
ordinary `EXCHANGE_AMMO_AT_POINT` path.

`claim_periodic_ammo` is not part of the current official `0x0120` sentry
autonomous decision command field set. Phase 1 keeps it disabled.

## Fix Status

- [x] P0 fixed: ordinary `EXCHANGE_AMMO_AT_POINT` no longer treats fortress as
  a legal normal exchange point.
- [x] P1 fixed: `CLAIM_PERIODIC_AMMO` is no longer connected in the main BT XML.
- [x] P2 fixed: `sentry_bt_sim` ordinary exchange logic no longer treats
  fortress as a normal exchange point.
- [ ] Control-side official `0x0301` / `0x0120` monotonic counter loop.

## Implementation Notes

- `on_fortress` remains in `RobotContext` and simulator input. It is still
  available for tactical fortress scenarios and fortress-specific rule logic.
- `ClaimPeriodicAmmoNode` remains registered as a local extension, but the main
  `RuleActionSubtree` routes the unused switch case to `NoRuleAction`.
