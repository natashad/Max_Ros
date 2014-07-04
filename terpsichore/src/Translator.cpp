#include "Translator.h"
#include <cmath>
#include <iostream>

void Conductor::calculatePosition(Transport tr){
	Position p = tr.position;
	TimeSignature ts = tr.timeSignature;
	double tempo = (double)tr.tempo;
	
	double t = (double)(p.beat - 1) + (double)p.unit/(double)p.resolution;
	
	if(ts.beatsPerBar == 4){
		//conduct in 4
		//z
		double A = 0.35;
		double B = 0.4;
		double C = 0.15;
		double D = 1.0;
	
		if(t < 2.0){
			z = (A/2.0)*(1.0 - cos(2.0*M_PI*t));
		}else if(t < 2.5){
			z = -16.0*B*pow((t - 2.0),3) + 12.0*B*pow((t-2.0),2);
		}else if(t < 3.0){
			z = 16.0*(B-C)*pow((t - 2.5),3) - 12.0*(B-C)*pow((t-2.5),2) + B;
		}else if(t < 3.5){
			z = 16.0*(C-D)*pow((t - 3.0),3) - 12.0*(C-D)*pow((t-3.0),2) + C;		
		}else if(t < 4.0){
			z = 16.0*D*pow((t-3.5),3) - 12.0*D*pow((t-3.5),2) + D;
		}
	
		//y
		double L = -0.7;
		double R = 0.7;
	
		if(t < 0.3){
			y = 0.0;
		}else if(t < 1.3){
			y = -2.0*L*pow((t - 0.3), 3) + 3.0*L*pow((t-0.3), 2);
		}else if(t < 2.3){
			y = 2.0*(L-R)*pow((t-1.3),3) - 3.0*(L-R)*pow((t-1.3),2) + L;
		}else if(t < 3.5){
			y = (125.0/108.0)*R*pow((t-2.3),3) - (25.0/12.0)*R*pow((t-2.3),2) + R;
		}else if(t < 4.0){
			y = 0.0;
		}
	}else if(ts.beatsPerBar == 3){
		//conduct in 3
		//z
		double A = 0.25;
		double B = 0.4;
		double C = 0.2;
		double D = 1.0;
	
		if(t < 1.0){
			z = (A/2.0)*(1.0 - cos(2.0*M_PI*t));
		}else if(t < 1.5){
			z = -16.0*B*pow((t - 1.0),3) + 12.0*B*pow((t-1.0),2);
		}else if(t < 2.0){
			z = 16.0*(B-C)*pow((t - 1.5),3) - 12.0*(B-C)*pow((t-1.5),2) + B;
		}else if(t < 2.5){
			z = 16.0*(C-D)*pow((t - 2.0),3) - 12.0*(C-D)*pow((t-2.0),2) + C;		
		}else if(t < 3.0){
			z = 16.0*D*pow((t-2.5),3) - 12.0*D*pow((t-2.5),2) + D;
		}
	
		//y
		double L = -0.3;
		double R = 0.8;
	
		if(t < 0.5){
			y = -16.0*L*pow(t, 3) + 12.0*L*pow(t, 2);
		}else if(t < 1.5){
			y = 2.0*(L-R)*pow((t - 0.5), 3) - 3.0*(L-R)*pow((t-0.5), 2) + L;
		}else if(t < 2.5){
			y = 2.0*R*pow((t-1.5),3) - 3.0*R*pow((t-1.5),2) + R;
		}else if(t < 3.0){
			y = 0.0;
		}
	}else if(ts.beatsPerBar == 2){
		//conduct in 2
		//z
		double A = 0.4;
		double B = 1.0;
		
		if(t < 1.0){
			z = (A/2.0)*(1.0 - cos(2.0*M_PI*t));
		}else if(t < 2.0){
			z = (B/2.0)*(1.0 - cos(2.0*M_PI*t));
		}
		
		//y
		double R = 0.4;
		
		if(t < 1.5){
			y = (R/2.0)*(1.0 - cos((4.0/3.0)*M_PI*t));
		}else if(t < 2.0){
			y = 0.0;
		}
	}
	x = 0.0;
}