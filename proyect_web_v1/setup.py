from setuptools import setup, Extension
import pybind11

# Define el módulo que compilará el archivo C++
module = Extension(
    'algoritmos_robot',  # Nombre del módulo que se va a generar
    sources=['algoritmos_robot.cpp'],  # Archivo fuente C++
    include_dirs=[pybind11.get_include()]  # Incluir pybind11
)

# Configuración de la compilación
setup(
    name='algoritmos_robot',  # Nombre del paquete
    version='1.0',
    ext_modules=[module],
)