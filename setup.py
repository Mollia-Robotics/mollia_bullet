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
    extra_compile_args=['-fpermissive'],
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
    data_files=[('.', ['mollia_bullet.pyi'])],
    install_requires=['numpy'],
    extras_require={
        'visualization': [
            'mollia_window',
            'zengl',
        ],
    },
)
