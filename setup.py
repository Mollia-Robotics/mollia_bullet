from setuptools import Extension, setup

core = Extension(
    'mollia_bullet',
    include_dirs=['bullet/src'],
    define_macros=[('BT_USE_DOUBLE_PRECISION', None)],
    sources=[
        'mollia_bullet.cpp',
        # 'bullet/src/btBulletCollisionAll.cpp',
        # 'bullet/src/btBulletDynamicsAll.cpp',
        # 'bullet/src/btLinearMathAll.cpp',
    ],
    extra_objects=[
        'build/temp.win-amd64-3.10/Release/bullet/src/btBulletCollisionAll.obj',
        'build/temp.win-amd64-3.10/Release/bullet/src/btBulletDynamicsAll.obj',
        'build/temp.win-amd64-3.10/Release/bullet/src/btLinearMathAll.obj',
    ],
)

setup(
    name='mollia_bullet',
    version='2.0.0',
    author='Mollia Zrt.',
    maintainer='Szabolcs Dombi',
    maintainer_email='cprogrammer1994@gmail.com',
    license='MIT',
    ext_modules=[core],
    packages=['_mollia_bullet'],
    install_requires=['numpy'],
)
