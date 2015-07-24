// COMPROBAR SI ES LA PRIMERA VEZ QUE ENTRA EL TARGET-->
		// CREAR PLAN PARA ALCANZAR EL TARGET USANDO GRAFO. ESE PLANNER TENDRA GRAFO (LO CREA O LO LEE), EL INNER Y EL TARGET
		// 1) BUSCA DE DONDE ESTAMOS AL GRAFO
		// 2) BUSCA DEL TARGET AL GRAFO
		// 3) BUSCAR CAMINO POR EL GRAFO DEVUELVE EL CAMINO COMO LISTA DE TARGET INYECTADOS AL PRINCIPIO.




	const float e1 = 0.0001 , e2 = 0.00000001, e3 = 0.0004, e4 = 0.f, t = pow(10, -3);
	const int kMax = 100;
	const QMat Identidad = QMat::identity(checkMotors().size()); //const QMat Identidad = QMat::identity(bodypart->getMotorList().size())

	// VARIABLES:
	int k=0, v=2, auxInt; //iterador, variable para descenso y un entero auxiliar
	QVec incrementos, aux; //vector de incrementos y vector auxiliar para guardar cambios
	QVec motors (checkMotors().size());//QVec motors (bodypart->getMotorList().size()); // lista de motores para rellenar el jacobiano.
	QVec angles = computeAngles(); // ángulos iniciales de los motores.

	// Creamos la matriz de pesos: Si antes hubo incrementos pequeños cuando se ejecutó por vez primera
	// ponemos TODAS las restricciones. Si no fue así, toma los pesos del usuario:
	QMat We;
	We = QMat::makeDiagonal(target.getTargetWeight());  //matriz de pesos para compensar milímietros con radianes.

	QVec error = We * computeErrorVector(target); //error de la posición actual con la deseada.
	QMat J = jacobian(motors);
	QMat H = J.transpose()*(We*J);// ERROR
	QVec g = J.transpose()*(error);
	bool stop = (g.maxAbs(auxInt) <= e1);
	bool smallInc = false;
	bool nanInc = false;
	float ro = 0;
	float n = t*H.getDiagonal().max(auxInt);

	while((stop==false) and (k<kMax) and (smallInc == false) and (nanInc == false))
	{
		k++;
		do{
			try
			{
				incrementos = (H + (Identidad*n)).invert() * g;
				for(int i=0; i<incrementos.size(); i++)
					if(isnan(incrementos[i])) 						///NAN increments
					{
						nanInc = true;
						bodypart->getTargetList()[0].setTargetFinalState(Target::NAN_INCS);
						break;
					}
				if(nanInc == true) break;
			}
			catch(QString str){ qDebug()<< __FUNCTION__ << __LINE__ << "SINGULAR MATRIX EXCEPTION"; }

			if(incrementos.norm2() <= e2)   ///Too small increments
			{
				stop = true;
				smallInc = true;
				bodypart->getTargetList()[0].setTargetFinalState(Target::LOW_INCS);
				break;
			}
			else
			{
				aux = angles-incrementos;
				computeFloatModule(aux, 2*M_PI); // NORMALIZAMOS

				if(outLimits(aux, motors) == true)		///COMPROBAR SI QUEDAN MOTORES LIBRES, SINO SALIR!!!!!!!!!!!
				{
					// Recalculamos el Jacobiano, el Hessiano y el vector g. El error es el mismo que antes
					// puesto que NO aplicamos los cambios (los ángulos nuevos).
					J = jacobian(motors);
					H = J.transpose()*(We*J);
					g = J.transpose()*(error);
				}
				updateAngles(aux); // Metemos los nuevos angles LUEGO HAY QUE DESHACER EL CAMBIO.
				
				//Check for sign of error derivative
				ro = ((error).norm2() - (We*computeErrorVector(target)).norm2()) /*/ (incrementos3*(incrementos3*n3 + g3))*/;
				
				//if sign is positive
				if(ro > 0)
				{
					motors.set((T)0);
					
					// Estamos descendiendo correctamente --> errorAntiguo > errorNuevo.
					stop = ((error).norm2() - (We*computeErrorVector(target)).norm2()) < e4*(error).norm2();
					angles = aux;
					
					// Recalculamos con nuevos datos.
					error = We*computeErrorVector(target);
					J = jacobian(motors);
					H = J.transpose()*(We*J);
					g = J.transpose()*(error);

					stop = (stop) or (g.maxAbs(auxInt)<=e1);
					n = n * std::max(1.f/3.f, (float)(1.f-pow(2*ro - 1,3)));
					v=2;
				}
				else
				{
					updateAngles(angles); //volvemos a los ángulos viejos.
					//increase landa values in regularization matrix
					n = n*v;
					v= 2*v;
				}
			}//fin else incrementos no despreciables.
		}while(ro<=0 and stop==false);
		stop = error.norm2() <= e3;
	}

	bodypart->getTargetList()[0].setTargetState(Target::FINISH);
	
	if (stop == true) 			bodypart->getTargetList()[0].setTargetFinalState(Target::LOW_ERROR);
	else if ( k>=kMax) 			bodypart->getTargetList()[0].setTargetFinalState(Target::KMAX);
	else if ( smallInc == true) bodypart->getTargetList()[0].setTargetFinalState(Target::LOW_INCS);
	else if ( nanInc == true) 	bodypart->getTargetList()[0].setTargetFinalState(Target::NAN_INCS);

	bodypart->getTargetList()[0].setTargetError(error);
	bodypart->getTargetList()[0].setTargetFinalAngles(angles);