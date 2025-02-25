
#ifndef VARS_H
#define VARS_H


//#define MARGEN 200		// recortar foto a 200 pixeles a izq y a der
//#define N_ULT_ARBOLES 8		// nro de arboles para ser tenido en cuenta
//#define N_ULT_ARBOLES 4		// nro de arboles para ser tenido en cuenta
//#define DISTANCIA_ARBOL 200	// hasta 200cm sería un arbol en la hilera

//#define CONSECUTIVOS 3		// si hay 3 fotos consecutivas con tronco 
				// entonces se lo considera "posible" arbol 
				
// para detectar un tronco analizamos verticalmente la imagen 
// en busca de una continuidad vertical de grises dentro de un umbral
double umbral_color = 20.0;	// ajustar segun contraste
double umbral_gris = 15.0;	// ajustar segun contraste


#define PIXELES_X_CM	(double)4.0	// calibracion: 2 pixeles = 1 cm
					// cuando la camara está a 100 cm 
					// del arbol


#endif  // VARS_H
