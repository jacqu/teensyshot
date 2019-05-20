/*
 *  AWPID:		Anti-windup PID API
 *  
 *  Note:     Best viewed using Arduino IDE with tab space = 2
 *
 *  Authors:  Jacques Gangloff
 *  Date:     May 2019
 */
 
// Includes
#include <math.h>

// Defines
#define AWPID_MAX_NB					10		// Maximum number of PIDs


// Main structure definition
typedef struct	{
	double Kp;				// Proportional gain
	double Ki;				// Integral gain
	double Kd;				// Derivative gain
	double f;					// Pole of the derivative term lowpass filter
	double Sat;				// Saturation
	double u0;				// Control signal before saturation
	double e0;				// Error signal
	double e1;				// Last sample error signal
	double ui0;				// Integral term
	double ui1;				// Last sample integral term
	double ud0;				// Derivative term
	double ud1;				// Last sample derivative term
	} AWPID_STRUCT;

// Global variables
AWPID_STRUCT	AWPID[AWPID_MAX_NB];
int 					awpid_nb = 0;

/**************************************************************************
*	rt_init_awpid : initialisation des parametres du PID.
*
*	* Nb : nombre de correcteurs
*	* Kp : coefficient proportionnel
*	* Ki : coefficient integrale
*	* Kd : coefficient derive
*	* f : pole du filtre du derivateur 
*			( 0 < f < 1 si f = 1, annulation de l'action derivee )
*	* Sat : valeur de la saturation (>0)   
*
*
*	C(z)=Kp[ 1 + Ki z / ( z - 1 ) + Kd( z - 1 ) / (z - f ) ]
*
**************************************************************************/
int rt_init_awpid( 	int		Nb,
					double	*Kp, 
					double	*Ki,
					double	*Kd,
					double	*f,
					double	*Sat
					 )	{
	
	int			i;
	
	if ( Nb > AWPID_MAX_NB )
		return 1;
	
	// Store the number of PIDs that are managed in global variable
	awpid_nb = Nb;
	
	for ( i = 0; i < Nb; i++ )	{
		
		/* Definition des coefficients des correcteurs */
		
		AWPID[i].Kp = Kp[i];
		AWPID[i].Ki = Ki[i];
		AWPID[i].Kd = Kd[i];
		AWPID[i].f = f[i];
		AWPID[i].Sat = Sat[i];
		
		/* Initialisation des varaibles du correcteur */
		
		AWPID[i].u0 = 0;
		
		AWPID[i].e0 = 0;
		AWPID[i].e1 = 0;
		
		AWPID[i].ui0 = 0;
		AWPID[i].ui1 = 0;
		
		AWPID[i].ud0 = 0;
		AWPID[i].ud1 = 0;
		}
	
	return 0;
	}

/**************************************************************************
*	rt_awpid : calcul de la commande.
**************************************************************************/
int rt_control( double *Reference, double *Measurement, double *Control )	{
	
	int		i;
	
	for ( i = 0; i < awpid_nb; i++ )	{
		
		/* Calcul de l'erreur */
		
		AWPID[i].e1 = AWPID[i].e0;
		AWPID[i].e0 = Reference[i] - Measurement[i];
		
		/* Calcul du terme derive */
		
		AWPID[i].ud1 = 	AWPID[i].ud0;
		AWPID[i].ud0 = 	AWPID[i].f * AWPID[i].ud1
						+ AWPID[i].Kp * AWPID[i].Kd * ( AWPID[i].e0 - AWPID[i].e1 );
		
		/* Calcul du terme integral avec anti-saturation */
		
		AWPID[i].ui1 = 	AWPID[i].ui0;
		
		/* Anti-saturation seulement s'il y a un terme intergral */
		
		if ( AWPID[i].Ki )	{
		
			AWPID[i].u0 = 	AWPID[i].Kp * AWPID[i].e0 +
							AWPID[i].ud0 +
							AWPID[i].ui1 + AWPID[i].Kp * AWPID[i].Ki * AWPID[i].e0;

			if ( fabs ( AWPID[i].u0 ) <= AWPID[i].Sat )	
				AWPID[i].ui0 = AWPID[i].ui1 + AWPID[i].Kp * AWPID[i].Ki * AWPID[i].e0;
				
			if ( AWPID[i].u0 > AWPID[i].Sat )	
				AWPID[i].ui0 = ( AWPID[i].Sat - 
									( AWPID[i].Kp * AWPID[i].e0 + AWPID[i].ud0 ) );

			if ( AWPID[i].u0 < -AWPID[i].Sat )	
				AWPID[i].ui0 = ( -AWPID[i].Sat - 
									( AWPID[i].Kp * AWPID[i].e0 + AWPID[i].ud0 ) );
			}
			
		/* Calcul de la commande saturee */
		
		Control[i] = 	AWPID[i].Kp * AWPID[i].e0 + 
						AWPID[i].ud0 + 
						AWPID[i].ui0;
						
		/* Saturation */
		
		if ( Control[i] > AWPID[i].Sat )
			Control[i] = AWPID[i].Sat;
			
		if ( Control[i] < -AWPID[i].Sat )
			Control[i] = -AWPID[i].Sat;
		}
	
	return 0;
	}
	
/**************************************************************************
*	Routine appelee a l'insertion du module.
**************************************************************************/
int init_module( void )	{
	
	/* Affichage d'un message */
	
	printk( KERN_INFO "awpid: loaded.\n" );
		
    return( 0 );
	}

/**************************************************************************
*	Routine appelee a la suppression du module.
**************************************************************************/
void cleanup_module( void )	{

    printk( KERN_INFO "awpid: unloaded.\n" );
	
    return;
	}
	
