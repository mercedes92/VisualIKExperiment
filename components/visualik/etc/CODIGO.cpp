bool SpecificWorker::correctTraslation	()
{
	qDebug()<<"\nCORRIGIENDO TRASLACION...";
	const float umbralMaxTime = 80, umbralMinTime = 10;
	const float umbralElapsedTime = 2, umbralError = 15;

	if(currentTarget.getRunTime()>umbralMaxTime and currentTarget.getRunTime()>umbralMinTime)
	{
		abortatraslacion = true;   
		qDebug()<<"Abort traslation";
		return false;
	}
	if(rightHand->getSecondsElapsed() >umbralElapsedTime)
		rightHand->setVisualPosewithInternal();
	
	QVec error 	= rightHand->getError();
	if (QVec::vec3(error.x(), error.y(), error.z()).norm2() < umbralError)
	{
		currentTarget.setState(Target::State::RESOLVED);
		qDebug()<<"done!: "<<QVec::vec3(error.x(), error.y(), error.z()).norm2();
		return true;
	}
	QVec tcorregida	= innerModel->transform("root", QVec::zeros(3),rightHand->getTip()) + error;
 	qDebug()<<"ERROR: "<<error<<"    norma: "<<QVec::vec3(error.x(), error.y(), error.z()).norm2();
	qDebug()<<"\nVISUAL: "<<rightHand->getVisualPose()<<"\nTARGET: "<<currentTarget.getPose()<<"\nINTERNAL: "<<innerModel->transform("root", QVec::zeros(3),rightHand->getTip());	
	WeightVector weights; //pesos a cero
	weights.x = 1;     weights.y = 1;    weights.z = 1;
	weights.rx = 0;    weights.ry = 0;   weights.rz = 0;
	
	QVec correccionFinal = QVec::vec6(0,0,0,0,0,0);
	correccionFinal.inject(tcorregida,0);
	correccionFinal.inject(QVec::vec3(currentTarget.getPose().rx(), currentTarget.getPose().ry(), currentTarget.getPose().rz()),3);	
	qDebug()<<"CORREGIDA FINAL: "<<correccionFinal;
	correctedTarget.setPose(correccionFinal);
	int identifier = inversekinematics_proxy->setTargetPose6D(currentTarget.getBodyPart(), correctedTarget.getPose6D(), weights);
	correctedTarget.setID(identifier);
	
	return false;
	// COMPROBAMOS EL ERROR: Si es miserable no hacemos nada y acabamos la corrección.
// 	QVec errorInv = rightHand->getErrorInverse();
// 	qDebug()<<"ErrorI: "<<errorInv<<"         Error d: "<<rightHand->getError();
// 	if (QVec::vec3(errorInv.x(), errorInv.y(), errorInv.z()).norm2() < umbralError)
// 	{
// 		currentTarget.setState(Target::State::RESOLVED);
// 		qDebug()<<"done!";
// 		return true;
// 	}
// 
// 	QVec errorInvP = QVec::vec3(errorInv(0), errorInv(1), errorInv(2));
// 	QVec errorInvPEnAbsoluto = innerModel->getRotationMatrixTo("root", rightHand->getTip())*errorInvP;
// 	qDebug()<<"Error T: "<<QVec::vec3(errorInv.x(), errorInv.y(),errorInv.z()).norm2();
// 
// 	QVec poseCorregida = innerModel->transform("root", rightHand->getTip()) + errorInvPEnAbsoluto;
// 	QVec correccionFinal = QVec::vec6(0,0,0,0,0,0);
// 	correccionFinal.inject(poseCorregida,0);
// 	correccionFinal.inject(QVec::vec3(currentTarget.getPose().rx(), currentTarget.getPose().ry(), currentTarget.getPose().rz()),3);	
// 	correctedTarget.setPose(correccionFinal);
// 	qDebug()<<"Posicion  corregida: "<<correctedTarget.getPose();
// 
// 	innerModel->updateTransformValues("corr_hand", correctedTarget.getPose().x(),   correctedTarget.getPose().y(), correctedTarget.getPose().z(), 
// 									               correctedTarget.getPose().rx(), correctedTarget.getPose().ry(), correctedTarget.getPose().rz());
// 	//Llamamos al BIK con el nuevo target corregido y esperamos
// 	WeightVector weights; //pesos a cero
// 	weights.x = 1;     weights.y = 1;    weights.z = 1;
// 	weights.rx = 0;    weights.ry = 0;   weights.rz = 0;
// 	
// 	int identifier = inversekinematics_proxy->setTargetPose6D(currentTarget.getBodyPart(), correctedTarget.getPose6D(), weights);
// 	correctedTarget.setID(identifier);
// 	return false;
}
/**
 * \brief Metodo CORRECT ROTATION
 * Corrige la posicion de la mano en traslacion y en rotacion.
 * @return bool TRUE si el target esta perfecto o FALSE si no lo esta.
 */
bool SpecificWorker::correctRotation()
{
	qDebug()<<"\nCORRIGIENDO ROTACION...";
	const float umbralMaxTime = 80, umbralMinTime = 10;
	const float umbralElapsedTime = 2, umbralErrorT = 15, umbralErrorR=0.01;

	// If the hand's tag is lost we assume that the internal possition (according to the direct kinematics) is correct
	if (rightHand->getSecondsElapsed() > umbralElapsedTime)
		rightHand->setVisualPosewithInternal();
	
	// COMPROBAMOS EL ERROR:
	//QVec errorInv = rightHand->getErrorInverse();
	QVec error 	= rightHand->getError();
	if((QVec::vec3(error.x(), error.y(), error.z()).norm2()<umbralErrorT and QVec::vec3(error.rx(), error.ry(), error.rz()).norm2()<umbralErrorR)
		or
		(currentTarget.getRunTime()>umbralMaxTime and currentTarget.getRunTime()>umbralMinTime))
	{		
		file<<"P: ("      		<<currentTarget.getPose();
		file<<") ErrorVisual_T:"<<QVec::vec3(error.x(), error.y(), error.z()).norm2();
		file<<" ErrorVisual_R:" <<QVec::vec3(error.rx(), error.ry(), error.rz()).norm2();
		file<<" ErrorDirecto_T:" <<inversekinematics_proxy->getTargetState(currentTarget.getBodyPart(), correctedTarget.getID()).errorT;
		file<<" ErrorDirecto_R: "<<inversekinematics_proxy->getTargetState(currentTarget.getBodyPart(), correctedTarget.getID()).errorR;
		file<<" END: "    		<<currentTarget.getRunTime();
		file<<" WHY?: "			<<inversekinematics_proxy->getTargetState(currentTarget.getBodyPart(), correctedTarget.getID()).state<<endl;
		flush(file);
		
		if(currentTarget.getRunTime()>umbralMaxTime and currentTarget.getRunTime()>umbralMinTime))
		{
			abortarotacion = true;
			qDebug()<<"Abort rotation";
			return false;
		}
		else
		{
			currentTarget.setState(Target::State::RESOLVED);
			qDebug()<<"||-->TERMINADO!!!: "<<QVec::vec3(error.x(), error.y(), error.z()).norm2();
			return true;
		}
	}
	QVec tcorregida	= innerModel->transform("root", QVec::zeros(3),rightHand->getTip()) + error;
 	qDebug()<<"ERROR: "<<error<<"    norma: "<<QVec::vec3(error.x(), error.y(), error.z()).norm2();
	qDebug()<<"\nVISUAL: "<<rightHand->getVisualPose()<<"\nTARGET: "<<currentTarget.getPose()<<"\nINTERNAL: "<<innerModel->transform("root", QVec::zeros(3),rightHand->getTip());	
	WeightVector weights; //pesos a cero
	weights.x = 1;     weights.y = 1;    weights.z = 1;
	weights.rx = 0;    weights.ry = 0;   weights.rz = 0;
	
	QVec correccionFinal = QVec::vec6(0,0,0,0,0,0);
	correccionFinal.inject(tcorregida,0);
	correccionFinal.inject(QVec::vec3(currentTarget.getPose().rx(), currentTarget.getPose().ry(), currentTarget.getPose().rz()),3);	
	qDebug()<<"CORREGIDA FINAL: "<<correccionFinal;
	correctedTarget.setPose(correccionFinal);
	int identifier = inversekinematics_proxy->setTargetPose6D(currentTarget.getBodyPart(), correctedTarget.getPose6D(), weights);
	correctedTarget.setID(identifier);
	
	
	
	
	// Si el error es miserable no hacemos nada y acabamos la corrección. Para hacer la norma lo pasamos a vec6
	if (QVec::vec3(errorInv.x(), errorInv.y(), errorInv.z()).norm2()<umbralErrorT and QVec::vec3(errorInv.rx(), errorInv.ry(), errorInv.rz()).norm2()<umbralErrorR)
	{
		currentTarget.setState(Target::State::RESOLVED);
		qDebug()<<"done!";
		file<<"P: ("      <<currentTarget.getPose();
		file<<") ErrorVisual_T:"<<QVec::vec3(errorInv.x(), errorInv.y(), errorInv.z()).norm2();
		file<<" ErrorVisual_R:" <<QVec::vec3(errorInv.rx(), errorInv.ry(), errorInv.rz()).norm2();
		file<<" ErrorDirecto_T:" <<inversekinematics_proxy->getTargetState(currentTarget.getBodyPart(), correctedTarget.getID()).errorT;
		file<<" ErrorDirecto_R: "<<inversekinematics_proxy->getTargetState(currentTarget.getBodyPart(), correctedTarget.getID()).errorR;
		file<<" END: "    <<currentTarget.getRunTime()<<"-->"<<abortatraslacion<<","<<abortarotacion;
		file<<" WHY?: "<<inversekinematics_proxy->getTargetState(currentTarget.getBodyPart(), correctedTarget.getID()).state<<endl;
		flush(file);
		return true;
	}
	
	QVec errorInvP = QVec::vec3(errorInv(0), errorInv(1), errorInv(2));
	QVec errorInvPEnAbsoluto = innerModel->getRotationMatrixTo("root", rightHand->getTip())*errorInvP;
	qDebug()<<"Error T: "<<QVec::vec3(errorInv.x(), errorInv.y(),errorInv.z()).norm2();

	QVec poseCorregida = innerModel->transform("root", rightHand->getTip()) + errorInvPEnAbsoluto;
	QVec correccionFinal = QVec::vec6(0,0,0,0,0,0);
	correccionFinal.inject(poseCorregida,0);
	correccionFinal.inject(QVec::vec3(currentTarget.getPose().rx(), currentTarget.getPose().ry(), currentTarget.getPose().rz()),3);	
	correctedTarget.setPose(correccionFinal);
	qDebug()<<"Correccion final: "<<correctedTarget.getPose();

	innerModel->updateTransformValues("corr_hand", correctedTarget.getPose().x(),  correctedTarget.getPose().y(),   correctedTarget.getPose().z(), 
									              correctedTarget.getPose().rx(), correctedTarget.getPose().ry(), correctedTarget.getPose6D().rz);
	
	
	
	
	//Llamamos al BIK con el nuevo target corregido y esperamos
	WeightVector weights; //pesos a cero
	weights.x = 1;     weights.y = 1;    weights.z = 1;
	weights.rx = 0;    weights.ry = 0;   weights.rz = 0;
	
	int identifierAUX = inversekinematics_proxy->setTargetPose6D(currentTarget.getBodyPart(), correctedTarget.getPose6D(), weights);
	correctedTarget.setID(identifierAUX);
	
	while(inversekinematics_proxy->getTargetState(currentTarget.getBodyPart(), correctedTarget.getID()).finish == false);
	updateMotors(inversekinematics_proxy->getTargetState(currentTarget.getBodyPart(), correctedTarget.getID()).motors);
	
	
	
	
	//Llamamos al BIK con el nuevo target corregido y esperamos
	int identifier = inversekinematics_proxy->setTargetPose6D(currentTarget.getBodyPart(), correctedTarget.getPose6D(), currentTarget.getWeights6D());
	correctedTarget.setID(identifier);
	return false;