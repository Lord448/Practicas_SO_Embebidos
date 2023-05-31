#include <stdio.h>
#include <string.h>
#include <math.h>

int main(int argc, char const *argv[])
{
    FILE *archivo;

    archivo = fopen("sucesion_numerica.csv", "w");

    if(archivo == NULL){
        printf("Error al abrir el archivo deseado");
        return(-1);
    }

    double potenciaN, sucesion[100];
    char cadena[100];
    int n = 1;

    for(int i = 0; i< 20 ; i++){
        potenciaN = pow(-1*n, 3);
        sucesion[i] = 1/potenciaN;
        sprintf(cadena, "%f ", sucesion[i]);
        fprintf(archivo, "%s", cadena);
        n++;
    } 
    fprintf(archivo,"\n"); 
    fclose(archivo);
    
    return 0;
}