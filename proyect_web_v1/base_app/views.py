from django.shortcuts import render
from django.http import HttpResponse



# Create your views here.

def home_system(request):
    resultado = None  # Inicializa la variable resultado

    if request.method == "POST":
        resultado = tu_funcion()  

    return render(request, 'home.html', {'resultado': resultado})

def tu_funcion():
    return "¡Hola, la función se ejecutó correctamente!"