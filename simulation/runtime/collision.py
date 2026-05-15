"""Collision-guard config and contact helpers for ME403 sim backends.

Stdlib-only so imports stay light for --help smoke checks; PyBullet/MuJoCo use
these types from their own code paths.
"""

from __future__ import annotations

from dataclasses import dataclass, field
import os
import warnings


TRUE_VALUES = {"1", "true", "yes", "on", "enabled"}
FALSE_VALUES = {"0", "false", "no", "off", "disabled"}


@dataclass(frozen=True)
class CollisionGuardConfig:
    """Runtime collision-guard configuration."""

    enabled: bool = False
    stop_on_collision: bool = True
    check_self: bool = True
    check_environment: bool = False
    distance_threshold: float = 0.0
    ignored_name_prefixes: tuple[str, ...] = ("simlab_",)
    ignored_link_pairs: frozenset[tuple[str, str]] = field(default_factory=frozenset)

    @classmethod
    def from_value(cls, value: bool | "CollisionGuardConfig" | None) -> "CollisionGuardConfig":
        """Build config from constructor value plus DOBOT_COLLISION_GUARD."""
        if isinstance(value, CollisionGuardConfig):
            return value
        if value is None:
            value = env_flag("DOBOT_COLLISION_GUARD", default=False)
        return cls(enabled=bool(value))


@dataclass(frozen=True)
class CollisionContact:
    """Normalized collision/contact report from a simulator backend."""

    backend: str
    kind: str
    body_a: str
    body_b: str
    link_a: str | None = None
    link_b: str | None = None
    distance: float | None = None
    normal_force: float | None = None

    def summary(self) -> str:
        a = f"{self.body_a}:{self.link_a}" if self.link_a else self.body_a
        b = f"{self.body_b}:{self.link_b}" if self.link_b else self.body_b
        dist = "" if self.distance is None else f" distance={self.distance:.4g}"
        force = "" if self.normal_force is None else f" force={self.normal_force:.4g}"
        return f"{self.kind} contact {a} <-> {b}{dist}{force}"


def env_flag(name: str, default: bool = False) -> bool:
    """Parse an environment variable as a boolean flag."""
    value = os.environ.get(name)
    if value is None:
        return default
    normalized = value.strip().lower()
    if normalized in TRUE_VALUES:
        return True
    if normalized in FALSE_VALUES:
        return False
    if normalized != "":
        warnings.warn(
            f"{name}={value!r} is not a recognized boolean "
            f"(expected one of {sorted(TRUE_VALUES | FALSE_VALUES)!r}); "
            f"using default={default!r}.",
            UserWarning,
            stacklevel=2,
        )
    return default


def normalized_pair(a: str | int | None, b: str | int | None) -> tuple[str, str]:
    """Return an order-independent pair key."""
    left = "" if a is None else str(a)
    right = "" if b is None else str(b)
    return tuple(sorted((left, right)))


def should_ignore_contact(
    link_a: str | int | None,
    link_b: str | int | None,
    config: CollisionGuardConfig,
) -> bool:
    """Return True for known-benign/self-helper contacts."""
    a = "" if link_a is None else str(link_a)
    b = "" if link_b is None else str(link_b)
    if not config.check_environment and (a == "world" or b == "world"):
        return True
    if any(a.startswith(prefix) or b.startswith(prefix) for prefix in config.ignored_name_prefixes):
        return True
    return normalized_pair(a, b) in config.ignored_link_pairs


def contact_summaries(contacts: list[CollisionContact], limit: int = 3) -> str:
    """Human-readable compact contact list."""
    if not contacts:
        return "no contacts"
    shown = "; ".join(c.summary() for c in contacts[:limit])
    if len(contacts) > limit:
        shown += f"; +{len(contacts) - limit} more"
    return shown
