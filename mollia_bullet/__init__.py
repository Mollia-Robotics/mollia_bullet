import numpy as np

from . import core as _imp

from typing import List, Tuple, Dict, Any, Callable, Union

StiffnessT = Tuple[float, float, bool]
ConstraintT = Union['Hinge', 'Fixed']
BodyT = Union['Box', 'Sphere']

VecT = Tuple[float, float, float]
QuatT = Tuple[float, float, float, float]
BasisT = Tuple[float, float, float, float, float, float, float, float, float]
ContactT = Tuple[BodyT, VecT, VecT, VecT, float, float]
TransformT = Tuple[VecT, Union[QuatT, BasisT]]


class RBody:
    __slots__ = ['_obj', '_friction', '_stiffness', 'mass', 'world', 'constraints', 'groups', 'color', 'visible', 'name', 'group', 'mask']

    def __init__(self):
        self._obj:Any = None
        self._friction:VecT = None
        self._stiffness:StiffnessT = None
        self.mass:Any = None
        self.world:Any = None
        self.constraints:Any = None
        self.groups:Any = None
        self.color:VecT = None
        self.visible:bool = None
        self.name:str = None
        self.group:int = None
        self.mask:int = None

    @property
    def origin(self) -> VecT:
        '''
        '''
        return self._obj.origin

    @origin.setter
    def origin(self, value):
        self._obj.origin = value

    @property
    def basis(self) -> BasisT:
        '''
        '''
        return self._obj.basis

    @basis.setter
    def basis(self, value):
        self._obj.basis = value

    @property
    def stiffness(self) -> StiffnessT:
        '''
        '''
        return self._stiffness

    @stiffness.setter
    def stiffness(self, value:StiffnessT):
        self._obj.stiffness = value

    @property
    def friction(self) -> VecT:
        '''
        '''
        return self._friction

    @friction.setter
    def friction(self, value:VecT):
        self._obj.friction = value

    @property
    def contact_stiffness(self):
        import warnings
        warnings.warn('body.contact_stiffness -> body.stiffness')
        return self._stiffness[0]

    @contact_stiffness.setter
    def contact_stiffness(self, value):
        import warnings
        warnings.warn('body.contact_stiffness -> body.stiffness')
        self._obj.stiffness = (value, self._stiffness[1], self._stiffness[2])

    @property
    def contact_damping(self):
        import warnings
        warnings.warn('body.contact_damping -> body.stiffness')
        return self._stiffness[1]

    @contact_damping.setter
    def contact_damping(self, value):
        import warnings
        warnings.warn('body.contact_damping -> body.stiffness')
        self._obj.stiffness = (self._stiffness[0], value, self._stiffness[2])

    @property
    def contact_stiffness_flag(self):
        import warnings
        warnings.warn('body.contact_stiffness_flag -> body.stiffness')
        return self._stiffness[2]

    @contact_stiffness_flag.setter
    def contact_stiffness_flag(self, value):
        import warnings
        warnings.warn('body.contact_stiffness_flag -> body.stiffness')
        self._obj.stiffness = (self._stiffness[0], self._stiffness[1], value)

    @property
    def linear_friction(self) -> float:
        '''
        '''
        return self._friction[0]

    @linear_friction.setter
    def linear_friction(self, value:float):
        self._obj.friction = (value, self._friction[1], self._friction[2])

    @property
    def rolling_friction(self) -> float:
        '''
        '''
        return self._friction[1]

    @rolling_friction.setter
    def rolling_friction(self, value:float):
        self._obj.friction = (self._friction[0], value, self._friction[2])

    @property
    def spinning_friction(self) -> float:
        '''
        '''
        return self._friction[2]

    @spinning_friction.setter
    def spinning_friction(self, value:float):
        self._obj.friction = (self._friction[0], self._friction[1], value)

    def apply_force(self, force:VecT, origin:VecT=None):
        '''
            force is given in world coordinates
            origin is given in local coordinates
        '''
        self._obj.apply_force(force, origin)

    def apply_torque(self, torque:VecT):
        '''
        '''
        self._obj.apply_torque(torque)

    def contacts(self, other:BodyT=None, mask:int=-1, eps:float=1e-3, local:bool=False) -> List[ContactT]:
        '''
            returns [(other, ptA, ptB, normB, distance, impulse), ...]
        '''
        return self._obj.contacts(other, mask, eps, local)

    def penetration(self, obj:BodyT) -> float:
        '''
        '''
        return self._obj.penetration(obj)

    def config(self, config:Dict[str, Any]=None) -> Dict[str, Any]:
        '''
        '''
        return self._obj.config(config)

    def remove(self):
        '''
        '''
        self._obj.remove()


class Box(RBody):
    __slots__ = ['size']

    def __init__(self):
        super().__init__()
        self.size:VecT = None

    @property
    def width(self):
        import warnings
        warnings.warn('box.width -> box.size')
        return self.size[0]

    @property
    def length(self):
        import warnings
        warnings.warn('box.length -> box.size')
        return self.size[1]

    @property
    def height(self):
        import warnings
        warnings.warn('box.height -> box.size')
        return self.size[2]


class Sphere(RBody):
    __slots__ = ['radius']

    def __init__(self):
        super().__init__()
        self.radius:float = None


class Constraint:
    __slots__ = ['_obj', 'world', 'body_a', 'body_b', 'name']

    def __init__(self):
        self._obj:Any = None
        self.world:'World' = None
        self.body_a:BodyT = None
        self.body_b:BodyT = None
        self.name:str = None

    def remove(self):
        '''
        '''
        self._obj.remove()

    @property
    def pivots(self):
        '''
            pivots in world coordinates
        '''
        return self._obj.pivot()


class Hinge(Constraint):
    __slots__ = ['motor_control']

    def __init__(self):
        super().__init__()
        self.motor_control:'MotorControl' = None

    @property
    def max_impulse(self):
        import warnings
        warnings.warn('hinge.max_impulse -> motor_control.input_array')
        return None

    @max_impulse.setter
    def max_impulse(self, value):
        import warnings
        warnings.warn('hinge.max_impulse -> motor_control.input_array')
        pass


class Fixed(Constraint):
    __slots__ = []

    def __init__(self):
        super().__init__()


class SixDof(Constraint):
    __slots__ = []

    def __init__(self):
        super().__init__()


class Slider(Constraint):
    __slots__ = []

    def __init__(self):
        super().__init__()


class PointToPoint(Constraint):
    __slots__ = []

    def __init__(self):
        super().__init__()


class ConeTwist(Constraint):
    __slots__ = []

    def __init__(self):
        super().__init__()


class Gear(Constraint):
    __slots__ = []

    def __init__(self):
        super().__init__()


class World:
    __slots__ = ['_obj', '_iterations', '_time_step', '_gravity', 'main_group', 'names', 'constraints', 'groups', 'motor_controls', 'updaters', 'main_motor_control']

    def __init__(self):
        self._obj:Any = None
        self._iterations:int = None
        self._time_step:float = None
        self._gravity:VecT = None
        self.main_group:'Group' = None
        self.names:Dict[str, BodyT] = None
        self.constraints:List[ConstraintT] = None
        self.groups:List['Group'] = None
        self.motor_controls:List['MotorControl'] = None
        self.updaters:List[Callable] = None

    @property
    def motor_dt(self):
        import warnings
        warnings.warn('world.motor_dt -> use velocity motor')
        return None

    @motor_dt.setter
    def motor_dt(self, value):
        import warnings
        warnings.warn('world.motor_dt -> use velocity motor')
        pass

    def __getitem__(self, key) -> Union[BodyT, ConstraintT]:
        import warnings
        warnings.warn('do not use names')
        return self.names[key]

    @property
    def gravity(self) -> VecT:
        '''
        '''
        return self._gravity

    @gravity.setter
    def gravity(self, value:VecT):
        self._obj.gravity = value

    @property
    def time_step(self) -> float:
        '''
        '''
        return self._time_step

    @time_step.setter
    def time_step(self, value:float):
        self._obj.time_step = value

    @property
    def num_iterations(self) -> int:
        '''
        '''
        return self._iterations

    @num_iterations.setter
    def num_iterations(self, value:int):
        self._obj.iterations = value

    def simulate(self, *args):
        '''
        '''
        # TODO: remove
        if len(args) == 1 and hasattr(self, 'main_motor_control'):
            import warnings
            warnings.warn('do not use old simulate')
            self.main_motor_control.input_array[:, 1] = args[0]
        self._obj.simulate()

    def contacts_of(self, obj, eps=1e-3, local=False):
        import warnings
        warnings.warn('old contacts_of')
        return self._obj.contacts_between(0, obj, None, eps, local)

    def contacts_between(self, obj1, obj2, eps=1e-3, local=False):
        import warnings
        warnings.warn('old contacts_between')
        return self._obj.contacts_between(1, obj1, obj2, eps, local)

    def contacts_between2(self, obj1, obj2, eps=1e-3, local=False):
        import warnings
        warnings.warn('old contacts_between2')
        return self._obj.contacts_between2(obj1, obj2, eps, local)

    def mesh(self, *args, **kwargs) -> bytes:
        '''
        '''
        return self._obj.mesh()

    def helper(self) -> bytes:
        '''
        '''
        return self._obj.helper()

    def contact_helper(self) -> bytes:
        '''
        '''
        return self._obj.contact_helper()

    def box(self, mass:float, half_extents:VecT, origin:VecT, orientation:Union[QuatT, BasisT]=None, ref:BodyT=None, group:int=0, mask:int=0, name:str=None) -> 'Box':
        '''
        '''
        return self._obj.rigid_body(mass, ('box', *half_extents), origin, orientation, ref, group, mask, name)

    def sphere(self, mass:float, radius:float, origin:VecT, orientation:Union[QuatT, BasisT]=None, ref:BodyT=None, group:int=0, mask:int=0, name:str=None) -> 'Sphere':
        '''
        '''
        return self._obj.rigid_body(mass, ('sphere', radius), origin, orientation, ref, group, mask, name)

    def hinge(self, body_a:BodyT, body_b:BodyT, pivot:VecT, axis:VecT=None, ref:BodyT=None, collision:bool=True, iterations:int=-1, motor_cut:float=None, name:str=None) -> 'Hinge':
        '''
        '''
        if iterations != -1:
            import warnings
            warnings.warn('world.hinge(iterations=...) -> world.num_iterations = ...')
        if motor_cut is not None:
            import warnings
            warnings.warn('world.hinge(motor_cut=...) -> use velocity motor')
        if name is not None:
            import warnings
            warnings.warn('do not use names')
        return self._obj.constraint(body_a, body_b, ('old_hinge', pivot, axis), ref, collision, iterations, name)

    def fixed(self, body_a:BodyT, body_b:BodyT, pivot:VecT=None, ref:BodyT=None, collision:bool=True, iterations:int=-1, name:str=None) -> 'Fixed':
        '''
        '''
        if iterations != -1:
            import warnings
            warnings.warn('world.fixed(iterations=...) -> world.num_iterations = ...')
        if name is not None:
            import warnings
            warnings.warn('do not use names')
        return self._obj.constraint(body_a, body_b, ('old_fixed', pivot), ref, collision, iterations, name)

    def sixdof(self, body_a:BodyT, body_b:BodyT, frame_a:TransformT=None, frame_b:TransformT=None, rot_order:int=0, ref:BodyT=None, collision:bool=True, iterations:int=-1, name:str=None) -> 'Fixed':
        '''
        '''
        if iterations != -1:
            import warnings
            warnings.warn('world.sixdof(iterations=...) -> world.num_iterations = ...')
        if name is not None:
            import warnings
            warnings.warn('do not use names')
        return self._obj.constraint(body_a, body_b, ('sixdof', frame_a, frame_b, rot_order), ref, collision, iterations, name)

    def motor_control(self, motors:List['Hinge']) -> 'MotorControl':
        '''
        '''
        return self._obj.motor_control(motors)

    def group(self, bodies:List[BodyT]) -> 'Group':
        '''
        '''
        return self._obj.group(bodies)

    def add_updater(self, updater:Callable):
        '''
        '''
        self._obj._updaters.append(updater)

    def remove_updater(self, updater:Callable):
        '''
        '''
        self._obj._updaters.remove(updater)

    def destroy(self):
        '''
        '''
        # TODO: remove
        if hasattr(self, 'main_motor_control'):
            self.main_motor_control = None
        self._obj.destroy()

    # TODO: remove
    @property
    def rigid_bodies(self):
        import warnings
        warnings.warn('world.rigid_bodies -> world.main_group.bodies')
        return self.main_group.bodies

    # TODO: remove
    @property
    def active_motors(self):
        import warnings
        warnings.warn('world.active_motors -> world.motor_control(...)')
        return [x.name for x in self.main_motor_control.motors]

    # TODO: remove
    @active_motors.setter
    def active_motors(self, value):
        import warnings
        warnings.warn('world.active_motors -> world.motor_control(...)')
        if hasattr(self, 'main_motor_control'):
            raise Exception('active_motors already set')
        self.main_motor_control = self.motor_control([self.names[x] for x in value])

    # TODO: remove
    @property
    def motor_angles(self):
        import warnings
        warnings.warn('world.motor_angles -> motor_control.position()')
        return self.main_motor_control.position()


class Group:
    __slots__ = ['_obj', 'world', 'bodies']

    def __init__(self):
        self._obj:Any = None
        self.world:'World' = None
        self.bodies:List[BodyT] = None

    def save_state(self) -> bytes:
        return self._obj.save_state()

    def load_state(self, state:bytes):
        self._obj.load_state(state)

    def apply_transform(self, transform:List[TransformT]):
        self._obj.apply_transform(transform)

    def apply_force(self, forces:List[VecT]):
        self._obj.apply_force(forces)

    def apply_torque(self, torques:List[VecT]):
        self._obj.apply_torque(torques)

    def aabb(self) -> Tuple[VecT, VecT]:
        return self._obj.aabb()

    def transforms(self) -> bytes:
        return self._obj.transforms()

    def center_of_mass(self) -> VecT:
        return self._obj.center_of_mass()

    def color_mesh(self) -> bytes:
        return self._obj.color_mesh()

    def remove(self):
        self._obj.remove()


class MotorControl:
    __slots__ = ['_obj', 'world', 'motors', 'input_mem', 'input_array']

    def __init__(self):
        self._obj:Any = None
        self.world:'World' = None
        self.motors:List[ConstraintT] = None
        self.input_mem:memoryview = None
        self.input_array:List[float] = None

    def position(self) -> List[float]:
        return self._obj.position()

    def velocity(self) -> List[float]:
        return self._obj.velocity()

    def reset(self):
        self._obj.reset()

    def remove(self):
        self._obj.remove()


def world(time_step=1 / 60, motor_dt=None, gravity=None, iterations=10, use_old_solver_mode=True, randomize_solver=False) -> World:
    if motor_dt is not None:
        import warnings
        warnings.warn('world.motor_dt -> use velocity motor')
    return _imp.world(time_step, gravity, iterations, use_old_solver_mode, randomize_solver)
