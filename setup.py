from setuptools import Extension, setup

core = Extension(
    'mollia_bullet',
    include_dirs=['bullet/src'],
    define_macros=[('BT_USE_DOUBLE_PRECISION', None)],
    sources=[
        'mollia_bullet.cpp',
        'bullet/src/btBulletCollisionAll.cpp',
        'bullet/src/btBulletDynamicsAll.cpp',
        'bullet/src/btLinearMathAll.cpp',
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
    py_modules=['_mollia_bullet'],
    install_requires=['numpy'],
)
