# Usa una imagen base de Python
FROM python:3.11

# Evita que Python genere archivos pyc
ENV PYTHONDONTWRITEBYTECODE 1
ENV PYTHONUNBUFFERED 1

# Crear el directorio de trabajo /app
RUN mkdir /app

# Establecer el directorio de trabajo
WORKDIR /app

# Instalar las dependencias necesarias para compilar código C++
RUN apt-get update && apt-get install -y \
    g++ \
    python3-dev \
    build-essential \
    cmake \
    libproj-dev \
    gdal-bin \
    && rm -rf /var/lib/apt/lists/*

# Instalar pybind11 y setuptools
RUN pip install pybind11 setuptools

# Copiar solo requirements.txt primero para aprovechar el caché de Docker
COPY ./requirements.txt /app/requirements.txt
RUN pip install -r requirements.txt

# Instalar daphne para Django Channels (si es necesario)
RUN pip install -U "channels[daphne]"

# Copiar todo el código del proyecto Django al contenedor
COPY . /app

# Compilar el módulo C++ usando setup.py (esto se añade al Dockerfile)
RUN python setup.py build_ext --inplace

# Configurar el servidor Gunicorn para correr la aplicación Django
CMD ["gunicorn", "-c", "config/gunicorn/conf.py", "--bind", ":8000", "--chdir", "proyect_web_v1", "proyect_web_v1.wsgi.application"]
