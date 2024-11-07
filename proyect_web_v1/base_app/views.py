from django.shortcuts import render
from django.http import HttpResponse
import algoritmos_robot 
from django.http import JsonResponse


def home_system(request):
    resultado = None  # Inicializa la variable resultado

    if request.method == "POST":
        resultado = tu_funcion()  

    return render(request, 'home.html', {'resultado': resultado})

def tu_funcion():
    return "¡Hola, la función se ejecutó correctamente!"


def send_data(request):
    return render(request, 'Home_Send/home_send.html')


def recibe_data(request):
    return render(request,'Home_recibe/home_recibe.html')





#Funciones de Origen C++ 
def suma_view(request):
    a = 5
    b = 3
    resultado = algoritmos_robot.suma(a, b)  # Usa la función suma del módulo C++
    return JsonResponse({'resultado': resultado})


def send_data(request):
    data_send=algoritmos_robot.enviar_mensaje_periodico()
    return JsonResponse({'resultado':data_send})







def calcular_grados_view(request):
    # Valores de ejemplo (pueden ser recibidos a través de un formulario o parámetros GET/POST)
    altitud = 10
    asimuth = 10
    roll = 10

    # Llamar a la función C++ desde Python (que devuelve una tupla)
    g_altitude, g_asimut, g_roll = algoritmos_robot.calcular_grados(altitud, asimuth, roll)

    # Retornar los resultados a una plantilla o a una respuesta JSON
    return JsonResponse({
        'GAltitude': g_altitude,
        'GAsimut': g_asimut,
        'GRoll': g_roll
    })

