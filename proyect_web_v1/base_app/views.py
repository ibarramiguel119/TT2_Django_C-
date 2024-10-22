from django.shortcuts import render
from django.http import HttpResponse
import algoritmos_robot 
from django.http import JsonResponse


# Create your views here.

def home_system(request):
    resultado = None  # Inicializa la variable resultado

    if request.method == "POST":
        resultado = tu_funcion()  

    return render(request, 'home.html', {'resultado': resultado})

def tu_funcion():
    return "¡Hola, la función se ejecutó correctamente!"


def suma_view(request):
    a = 5
    b = 3
    resultado = algoritmos_robot.suma(a, b)  # Usa la función suma del módulo C++
    return JsonResponse({'resultado': resultado})


def send_data(request):
    return render(request, 'Home_Send/home_send.html')


def recibe_data(request):
    return render(request,'Home_recibe/home_recibe.html')

