FROM python:3.11

ENV PYTHONDONTWRITEBYTECODE 1
ENV PYTHONUNBUFFERED 1

RUN mkdir /app
# Directorio de trabajo
WORKDIR /app


# Install GEOS library and set environment variables for GEOS

RUN apt-get update && apt-get install -y \
    binutils \
    libproj-dev \
    gdal-bin \
    && rm -rf /var/lib/apt/lists/*



# Instalación de dependencias
COPY ./requirements.txt /app/requirements.txt
RUN pip install -r requirements.txt
RUN pip install -U channels["daphne"]



# Copiar elpip install --upgrade pip código de la aplicación
COPY . /app


CMD ["gunicorn", "-c", "config/gunicorn/conf.py", "--bind", ":8000", "--chdir", "proyect_web_v1", "proyect_web_v1.wsgi.application"]
