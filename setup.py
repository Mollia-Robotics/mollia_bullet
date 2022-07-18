from setuptools import Extension, setup

macros = [
    ('BT_DISABLE_CAPSULE_CAPSULE_COLLIDER', 1),
    ('BT_USE_DOUBLE_PRECISION', None),
    ('BT_NO_PROFILE', None),
]

bullet_source = [
    'mollia_bullet/core/bullet/src/Bullet3Collision/BroadPhaseCollision/b3DynamicBvh.cpp',
    'mollia_bullet/core/bullet/src/Bullet3Collision/BroadPhaseCollision/b3DynamicBvhBroadphase.cpp',
    'mollia_bullet/core/bullet/src/Bullet3Collision/BroadPhaseCollision/b3OverlappingPairCache.cpp',
    'mollia_bullet/core/bullet/src/Bullet3Collision/NarrowPhaseCollision/b3ConvexUtility.cpp',
    'mollia_bullet/core/bullet/src/Bullet3Collision/NarrowPhaseCollision/b3CpuNarrowPhase.cpp',
    'mollia_bullet/core/bullet/src/Bullet3Common/b3AlignedAllocator.cpp',
    'mollia_bullet/core/bullet/src/Bullet3Common/b3Logging.cpp',
    'mollia_bullet/core/bullet/src/Bullet3Common/b3Vector3.cpp',
    'mollia_bullet/core/bullet/src/Bullet3Geometry/b3ConvexHullComputer.cpp',
    'mollia_bullet/core/bullet/src/Bullet3Geometry/b3GeometryUtil.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/BroadphaseCollision/btAxisSweep3.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/BroadphaseCollision/btBroadphaseProxy.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/BroadphaseCollision/btCollisionAlgorithm.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/BroadphaseCollision/btDbvt.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/BroadphaseCollision/btDbvtBroadphase.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/BroadphaseCollision/btDispatcher.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/BroadphaseCollision/btOverlappingPairCache.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/BroadphaseCollision/btQuantizedBvh.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/BroadphaseCollision/btSimpleBroadphase.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionDispatch/btActivatingCollisionAlgorithm.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionDispatch/btBox2dBox2dCollisionAlgorithm.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionDispatch/btBoxBoxCollisionAlgorithm.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionDispatch/btBoxBoxDetector.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionDispatch/btCollisionDispatcher.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionDispatch/btCollisionObject.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionDispatch/btCollisionWorld.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionDispatch/btCollisionWorldImporter.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionDispatch/btCompoundCompoundCollisionAlgorithm.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionDispatch/btConvex2dConvex2dAlgorithm.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionDispatch/btConvexConcaveCollisionAlgorithm.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionDispatch/btConvexConvexAlgorithm.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionDispatch/btConvexPlaneCollisionAlgorithm.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionDispatch/btEmptyCollisionAlgorithm.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionDispatch/btGhostObject.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionDispatch/btHashedSimplePairCache.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionDispatch/btInternalEdgeUtility.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionDispatch/btManifoldResult.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionDispatch/btSimulationIslandManager.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionDispatch/btSphereBoxCollisionAlgorithm.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionDispatch/btSphereSphereCollisionAlgorithm.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionDispatch/btSphereTriangleCollisionAlgorithm.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionDispatch/btUnionFind.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionDispatch/SphereTriangleDetector.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btBox2dShape.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btBoxShape.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btBvhTriangleMeshShape.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btCapsuleShape.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btCollisionShape.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btCompoundShape.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btConcaveShape.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btConeShape.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btConvex2dShape.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btConvexHullShape.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btConvexInternalShape.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btConvexPointCloudShape.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btConvexPolyhedron.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btConvexShape.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btConvexTriangleMeshShape.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btCylinderShape.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btEmptyShape.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btHeightfieldTerrainShape.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btMinkowskiSumShape.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btMultimaterialTriangleMeshShape.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btMultiSphereShape.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btOptimizedBvh.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btPolyhedralConvexShape.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btShapeHull.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btSphereShape.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btStaticPlaneShape.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btStridingMeshInterface.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btTetrahedronShape.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btTriangleBuffer.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btTriangleCallback.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btTriangleIndexVertexArray.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btTriangleIndexVertexMaterialArray.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btTriangleMesh.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btTriangleMeshShape.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/CollisionShapes/btUniformScalingShape.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/NarrowPhaseCollision/btContinuousConvexCollision.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/NarrowPhaseCollision/btConvexCast.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/NarrowPhaseCollision/btGjkConvexCast.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/NarrowPhaseCollision/btGjkEpa2.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/NarrowPhaseCollision/btGjkPairDetector.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/NarrowPhaseCollision/btMinkowskiPenetrationDepthSolver.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/NarrowPhaseCollision/btPersistentManifold.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/NarrowPhaseCollision/btPolyhedralContactClipping.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/NarrowPhaseCollision/btRaycastCallback.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/NarrowPhaseCollision/btSubSimplexConvexCast.cpp',
    'mollia_bullet/core/bullet/src/BulletCollision/NarrowPhaseCollision/btVoronoiSimplexSolver.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/ConstraintSolver/btConeTwistConstraint.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/ConstraintSolver/btContactConstraint.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/ConstraintSolver/btFixedConstraint.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/ConstraintSolver/btGearConstraint.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/ConstraintSolver/btHinge2Constraint.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/ConstraintSolver/btHingeConstraint.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/ConstraintSolver/btNNCGConstraintSolver.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/ConstraintSolver/btPoint2PointConstraint.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/ConstraintSolver/btSliderConstraint.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/ConstraintSolver/btSolve2LinearConstraint.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/ConstraintSolver/btTypedConstraint.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/ConstraintSolver/btUniversalConstraint.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/Dynamics/btDiscreteDynamicsWorld.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/Dynamics/btDiscreteDynamicsWorldMt.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/Dynamics/btRigidBody.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/Dynamics/btSimpleDynamicsWorld.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/Dynamics/btSimulationIslandManagerMt.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/Featherstone/btMultiBody.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/Featherstone/btMultiBodyConstraint.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/Featherstone/btMultiBodyConstraintSolver.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/Featherstone/btMultiBodyFixedConstraint.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/Featherstone/btMultiBodyJointMotor.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/Featherstone/btMultiBodyPoint2Point.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/Featherstone/btMultiBodySliderConstraint.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/MLCPSolvers/btDantzigLCP.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/MLCPSolvers/btLemkeAlgorithm.cpp',
    'mollia_bullet/core/bullet/src/BulletDynamics/MLCPSolvers/btMLCPSolver.cpp',
    'mollia_bullet/core/bullet/src/LinearMath/btAlignedAllocator.cpp',
    'mollia_bullet/core/bullet/src/LinearMath/btConvexHull.cpp',
    'mollia_bullet/core/bullet/src/LinearMath/btConvexHullComputer.cpp',
    'mollia_bullet/core/bullet/src/LinearMath/btGeometryUtil.cpp',
    'mollia_bullet/core/bullet/src/LinearMath/btPolarDecomposition.cpp',
    'mollia_bullet/core/bullet/src/LinearMath/btQuickprof.cpp',
    'mollia_bullet/core/bullet/src/LinearMath/btThreads.cpp',
    'mollia_bullet/core/bullet/src/LinearMath/btVector3.cpp',
]

core = Extension(
    'mollia_bullet.core',
    include_dirs=['mollia_bullet/core/bullet/src'],
    define_macros=macros,
    sources=[
        'mollia_bullet/core/constraint.cpp',
        'mollia_bullet/core/group.cpp',
        'mollia_bullet/core/color_mesh.cpp',
        'mollia_bullet/core/mollia_bullet.cpp',
        'mollia_bullet/core/motor_control.cpp',
        'mollia_bullet/core/rigid_body.cpp',
        'mollia_bullet/core/world.cpp',
    ] + bullet_source,
    depends=[
        'mollia_bullet/core/common.hpp',
        'mollia_bullet/core/constraint.hpp',
        'mollia_bullet/core/group.hpp',
        'mollia_bullet/core/color_mesh.hpp',
        'mollia_bullet/core/motor_control.hpp',
        'mollia_bullet/core/rigid_body.hpp',
        'mollia_bullet/core/world.hpp',
        'setup.py',
    ],
)

setup(
    name='mollia_bullet',
    version='1.0.1',
    author='Mollia Zrt.',
    license='MIT',
    packages=['mollia_bullet'],
    ext_modules=[core],
    install_requires=['numpy', 'zengl'],
)
