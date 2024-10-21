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




def send_data(request):
    return render(request, 'Home_Send/home_send.html')


def recibe_data(request):
    return render(request,'Home_recibe/home_recibe.html')

