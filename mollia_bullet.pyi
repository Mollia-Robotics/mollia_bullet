from typing import List, Literal, Tuple, TypedDict

IDENTITY: Transform

Vector = Tuple[float, float, float]
Quaternion = Tuple[float, float, float, float]


class EmptyShape(TypedDict, total=False):
    type: Literal['empty']


class BoxShape(TypedDict, total=False):
    type: Literal['box']
    size: Tuple[float, float, float]


class SphereShape(TypedDict, total=False):
    type: Literal['sphere']
    radius: float


class CapsuleShape(TypedDict, total=False):
    type: Literal['capsule']
    radius: float
    height: float


class CylinderShape(TypedDict, total=False):
    type: Literal['cylinder']
    radius: float
    height: float


class ConeShape(TypedDict, total=False):
    type: Literal['cone']
    radius: float
    height: float


class ConvexHullShape(TypedDict, total=False):
    type: Literal['convexhull']
    vertices: List[Tuple[float, float, float]]


class MultiSphereShape(TypedDict, total=False):
    type: Literal['multisphere']
    spheres: List[Tuple[float, float, float, float]]


class ShapeAndTransform(TypedDict, total=False):
    transform: Transform
    shape: Shape


class MinkowskiSumShape(TypedDict, total=False):
    type: Literal['minkowski']
    a: ShapeAndTransform
    b: ShapeAndTransform


class CompoundShape(TypedDict, total=False):
    type: Literal['compound']
    children: List[ShapeAndTransform]


Shape = EmptyShape | BoxShape | SphereShape | CapsuleShape | CylinderShape | ConeShape | ConvexHullShape | MultiSphereShape | MinkowskiSumShape | CompoundShape


class Transform:
    def position(self) -> Vector: ...
    def rotation(self) -> Quaternion: ...
    def inverse(self) -> Transform: ...


class RigidBody:
    mass: float
    transform: Transform


class Constraint:
    parent: RigidBody
    child: RigidBody

    def configure(self, dof: int, motor: bool = False, spring: bool = False, servo: bool = False,
        servo_target: float = 0.0, target_velocity: float = 0.0, max_motor_force: float = 0.0, stiffness: float = 0.0,
        damping: float = 0.0, bounce: float = 0.0, lower_limit: float = 0.0, upper_limit: float = 0.0,
        equilibrium_point: float = 0.0) -> None: ...


class World:
    def rigid_body(self, mass: float = 0.0, shape: Shape | None = None, transform: Transform = IDENTITY,
        friction: float = 0.0, spinning_friction: float = 0.0, rolling_friction: float = 0.0,
        group: int = 1, mask: int = 1) -> RigidBody: ...
    def constraint(self, parent: RigidBody, child: RigidBody, parent_pivot: Transform = IDENTITY,
        child_pivot: Transform = IDENTITY, rotation_order: str = 'xyz') -> Constraint: ...


def world(gravity: Vector) -> World: ...

def transform(position: Vector, rotation: Quaternion) -> Transform: ...
