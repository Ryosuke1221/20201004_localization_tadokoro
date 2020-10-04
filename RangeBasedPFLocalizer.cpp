#include "StdAfx.h"
#include "RangeBasedPFLocalizer.h"

float ** CRangeBasedPFLocalizer::m_fRangeMCLPro = NULL;
float ** CRangeBasedPFLocalizer::m_fRangeFPro = NULL;
float ** CRangeBasedPFLocalizer::m_fRangeALLPro = NULL;

CRangeBasedPFLocalizer::CRangeBasedPFLocalizer(void)
{
		m_RobotState.initialize();
		m_RobotState.setCovarianceDimension(3);
		m_AccumulatedMovement.r = 0.0;
		m_AccumulatedMovement.th = 0.0;
		m_dParticleSD = 0.0;
}


CRangeBasedPFLocalizer::~CRangeBasedPFLocalizer(void)
{

	delete  m_pParticles;
		double dMaxDistanceMM = (double)m_Param.dRangeMaxDist*1000.0;
		for( int i=0;i<(int)(dMaxDistanceMM/10.0)+1;i++)
		{
			free(CRangeBasedPFLocalizer::m_fRangeMCLPro[i]); 
		}
		free (CRangeBasedPFLocalizer::m_fRangeMCLPro);	

}

void CRangeBasedPFLocalizer::generateSensorModel()
{
	int i;
	float s;
	float fTemp;
	float fMaxValue;
	float fValueAtMaxRange;

	//GetPrivateProfileString("MCL","Sigma","1.0",buf,sizeof(buf),inifile);
	s = 1.0;//atof(buf);
	printf("Parameter loading from Parameters.ini....\n(Sigma = %.2f)\n", s);

	double dMaxDistanceMM = (double)m_Param.dRangeMaxDist*1000.0;


	// range-based MCL에서 거리차에 따른 확톩E?미리 계퍊E
	CRangeBasedPFLocalizer::m_fRangeMCLPro = (float**)calloc((int)(dMaxDistanceMM/10.0)+1,sizeof(float*));

	for(i=0;i<(int)(dMaxDistanceMM/10.0)+1;i++) {
		CRangeBasedPFLocalizer::m_fRangeMCLPro[i] = (float*)calloc((int)(dMaxDistanceMM/10.0)+1,sizeof(float));
	}	
	int nnn = (int)((dMaxDistanceMM/10.0)+1);
// 	fMaxValue = (float)0.05;
// 	
// 	for(i=0;i<(int)(dMaxDistanceMM/10.0) + 1;i++)  // expected distance	
// 	{
// 		for(int j=0;j<(int)(dMaxDistanceMM / 10.0)+1;j++)  // measured distance
// 		{	
// 			CRangeBasedPFLocalizer::m_fRangeMCLPro[i][j] = 0.001 + fMaxValue*exp(-((float)i*0.01-(float)j*0.01)*((float)i*0.01-(float)j*0.01)/2/s/s);
// 		}
// 	}

	CRangeBasedPFLocalizer::m_fRangeFPro = (float**)calloc(256, sizeof(float*));

	for (i = 0; i < 256; i++) {
		CRangeBasedPFLocalizer::m_fRangeFPro[i] = (float*)calloc(256, sizeof(float));
	}
	int mmm = 256;



	float sigma;
	for (int i = 0; i < (int)(dMaxDistanceMM / 10.0) + 1; i++)
	{	// expected distance
		sigma = 0.006f*(float)i*0.01f + 0.150f;
		for (int j = 0; j < (int)(dMaxDistanceMM / 10.0) + 1; j++)
		{	// measured distance
			//m_fRangeMCLPro[i][j] = 0.05*exp(-((float)i*0.01-(float)j*0.01)*((float)i*0.01-(float)j*0.01)/2/sigma/sigma)
			//+ 0.005*((float)m_dMaxDistance-(float)j*10.0)/(float)m_dMaxDistance;
			CRangeBasedPFLocalizer::m_fRangeMCLPro[i][j] = 1.0f / (sqrt(2.0f*(float)M_PI*sigma))*exp(-((float)i*0.01f - (float)j*0.01f)*((float)i*0.01f - (float)j*0.01f) / 2.0f / sigma / sigma)+ 0.005*((float)dMaxDistanceMM - (float)j*10.0f) / (float)dMaxDistanceMM;
			// 			if(((float)j*10.0)>((float)dMaxDistanceMM-300.0))
			// 				CRangeBasedPFLocalizer::m_fRangeMCLPro[i][j] = (float)0.10;
						// 모션모델의 확톩E갋㎏?과潭하여 퉩E㎏?맞춰주는 부분. (0~1.0사이 조절)*(가중치)
						//CRangeBasedPFLocalizer::m_fRangeMCLPro[i][j] *= (10.0 * 1.5);
		}
	}

	float sigma_h;
	for (int h = 0; h < 256; h++)
	{
		sigma_h = 8;
		for (int k = 0; k < 256; k++)
		{
			CRangeBasedPFLocalizer::m_fRangeFPro[h][k] = 1.0f / (sqrt(2.0f*(float)M_PI*sigma_h))*exp(-((float)h*0.01f - (float)k*0.01f)*((float)h*0.01f - (float)k*0.01f) / 2.0f / sigma_h / sigma_h);

		}

	}
	
	printf("Sensor model of MCL was completely generated.\n");
}

float CRangeBasedPFLocalizer::getFeatureValue(float x, float y)
{
	float feature = 0.;
	if (10 <= x && x <= 15 && 1 <= y && y <= 2)feature = 1.;

	return feature;
}


void CRangeBasedPFLocalizer::estimateState()
{
	// 1. prediction
	prediction();
	
 	if(isTimeToUpdate())
	{
		// 2. update
		update();
	
		Normalizing();

		// 3. resampling
		resampling();
	}
		
 	// 4. estimation
 	estimation();
}


// void CRangeBasedPFLocalizer::prediction()
// {
// 	double dEncoderDelta2[3];
// 	double dDeltaDistance, dDeltaTheta;
// 	int nCnt = 0;
// 	double s1,c1;
// 	double dPro;
// 	double dProTrans, dProTransRotate, dProRotate;
// 	double d1, d2, d3;
// 	double dNoiseTrans, dNoisexTrans, dNoiseyTrans,dNoiseTransRotate, dNoiseRotate;
// 	double dTransdistance;
// 	
// 	double	dMaxProForUpdate = 0.8;
// 	double dMinProForUpdate = 0.2;
// 	d1 = m_Param.dDeviationforTrans;
// 	d2 = m_Param.dDeviationforTransToRot;
// 	d3 = m_Param.dDeviationforRot;
// 
// 
// 	// Case that there are no encoder data, deactivating this function.
// 	if (m_DeltaPosition.x==0.0 && m_DeltaPosition.y==0.0 && m_DeltaOrientation.yaw==0.0) return;
// 
// 	double dDX = m_DeltaPosition.x*1000.0;
// 	double dDY = m_DeltaPosition.y*1000.0;
// 	double dDTh = m_DeltaOrientation.yaw;
// 	
// 	// Add noise. Motion model driven by Konolige
// 	int nX = 0;
// 	int nY = 0;
// 	for (int i=0; i<m_nParticleNum; i++) 
// 	{
// 		dTransdistance = sqrt(pow(dDX,2)+pow(dDY,2));			// 직선이동거리
// 		dNoiseTrans = CCalculus::getInstance()->getRandomValue(dTransdistance*d1);	// x 퉩EE오차는 x 퉩EE이동량에만 비례.
// 		dNoiseTransRotate = CCalculus::getInstance()->getRandomValue(dTransdistance*d2);	// 직선웝肌에 따른 각도오혖E 이동거리에 비례하여 증가한다.
// 		dNoiseRotate = CCalculus::getInstance()->getRandomValue(dDTh*d3);
// 		dDeltaDistance = dTransdistance + dNoiseTrans;
// 		dDeltaTheta = dDTh			// 각도 오차는,
// 		+ dNoiseTransRotate	// 직선이동에 의한 각도오차와
// 			+ dNoiseRotate;		// 회픸E?의한 각도오차를 흟E?구해진다.
// 
// 		s1 = sin(   m_pParticles[i].th + dDTh + dNoiseRotate ); // yaw
// 		c1 = cos(   m_pParticles[i].th + dDTh + dNoiseRotate  ); // yaw
// 
// 		// 오차가 포함된 엔코큱E정보를 이퓖E臼?샘플의 절큱E쪄Ⅰ갋상에서의 이동을 계퍊E
// 		// Calculation of the movement with encoder data and errors in the the absoulte coordinate
// 
// 		m_pParticles[i].x += (c1*dDeltaDistance)*0.001;
// 		m_pParticles[i].y += (s1*dDeltaDistance)*0.001;
// 		m_pParticles[i].th += dDeltaTheta;
// 		if (m_pParticles[i].th>M_PI) m_pParticles[i].th -=2*M_PI;
// 		else if (m_pParticles[i].th<-M_PI) m_pParticles[i].th +=2*M_PI;
// 
// 		// 이동량에 따른 확톩E갱신.
// 		// 엔코더로 예측한 값이 정확하다툈E엔코큱E변화량컖E퀋E訣濠?추가된 변화량이 일치해야 하므로,
// 		// 엔코큱E변화량컖E퀋E訣濠?추가된 변화량 사이의 차이가 작을수록 높은 확톩E?부여.
// 		dProTrans = CCalculus::getInstance()->getGaussianValue(dTransdistance*d1, dNoiseTrans);
// 		if (dProTrans<dMinProForUpdate)
// 			dProTrans = dMinProForUpdate;
// 		if (dProTrans>dMaxProForUpdate)
// 			dProTrans = dMaxProForUpdate;
// 		dProTransRotate = CCalculus::getInstance()->getGaussianValue(dTransdistance*d2, dNoiseTransRotate);
// 		if (dProTransRotate<dMinProForUpdate)
// 			dProTransRotate = dMinProForUpdate;
// 		if (dProTransRotate>dMaxProForUpdate)
// 			dProTransRotate = dMaxProForUpdate;
// 		dProRotate = CCalculus::getInstance()->getGaussianValue(dDTh*d3, dNoiseRotate);
// 		if (dProRotate<dMinProForUpdate)
// 			dProRotate = dMinProForUpdate;
// 		if (dProRotate>dMaxProForUpdate)
// 			dProRotate = dMaxProForUpdate;
// 
// 		m_pParticles[i].dPro *= dProTrans*dProTransRotate*dProRotate;
// 
// 
// 		nX = (int)(m_pParticles[i].x/m_pMap->getCellSize());
// 		nY = (int)(m_pParticles[i].y/m_pMap->getCellSize());
// 
// // 		if(			nX <= 0 || nX >= m_pMap->getMapSizeX() || nY <=0 || nY >= m_pMap->getMapSizeY() 			)
// // 		{
// // 			m_pParticles[i].dPro = 0.0;
// // 			continue;
// // 		}
// 			
// 
// 		if(m_pMap->get2DGridMap()[nX][nY] == CTWODMapState::UNKNOWN ||
// 			nX <= 0 || nX >= m_pMap->getMapSizeX() || nY <=0 || nY >= m_pMap->getMapSizeY() 			)
// 			m_pParticles[i].dPro = 0.0;
// 
// 	}
// 
// 
// }


void CRangeBasedPFLocalizer::prediction()
{
	double dEncoderDelta2[3];
	int nCnt = 0;
	double s1,c1;
	double dPro;
	double d1, d2, d3;
	double dNoise;
	d1 = m_Param.dDeviationforTrans;
	d2 = m_Param.dDeviationforTransToRot;
	d3 = m_Param.dDeviationforRot;


	// Case that there are no encoder data, deactivating this function.
	if (m_DeltaPosition.x==0.0 && m_DeltaPosition.y==0.0 && m_DeltaOrientation.yaw==0.0) return;

	double dDX = m_DeltaPosition.x*1000.0;
	double dDY = m_DeltaPosition.y*1000.0;
	double dDTh = m_DeltaOrientation.yaw;

	// Add noise. Motion model driven by Konolige
	int nX = 0;
	int nY = 0;
	for (int i=0; i<m_nParticleNum; i++) 
	{

		//cout <<"D: " <<  m_pParticles[i].th*R2D << endl;
		dNoise = CCalculus::getInstance()->getRandomValue(dDX*d1);	// x 퉩EE오차는 x 퉩EE이동량에만 비례.
		//		if (dNoise>0) dNoise /= 2.0;
		dEncoderDelta2[0] = dDX+ dNoise;
		dEncoderDelta2[1] = dDY;
		dEncoderDelta2[2] = dDTh							// Angular error
			+ CCalculus::getInstance()->getRandomValue(dDX*d2)	// x directio movement
			+ CCalculus::getInstance()->getRandomValue(dDTh*d3);

		s1 = sin(   m_pParticles[i].th + dEncoderDelta2[2]/2.0   ); // yaw
		c1 = cos(   m_pParticles[i].th + dEncoderDelta2[2]/2.0   ); // yaw


		m_pParticles[i].x += (c1*dEncoderDelta2[0] + (-s1)*dEncoderDelta2[1])*0.001;
		m_pParticles[i].y += (s1*dEncoderDelta2[0] + (c1)*dEncoderDelta2[1])*0.001;
		m_pParticles[i].th += dEncoderDelta2[2];
		if (m_pParticles[i].th>M_PI) m_pParticles[i].th -=2*M_PI;
		else if (m_pParticles[i].th<-M_PI) m_pParticles[i].th +=2*M_PI;

		nX = (int)(m_pParticles[i].x/m_pMap->getCellSize());
		nY = (int)(m_pParticles[i].y/m_pMap->getCellSize());

		// 		if(			nX <= 0 || nX >= m_pMap->getMapSizeX() || nY <=0 || nY >= m_pMap->getMapSizeY() 			)
		// 		{
		// 			m_pParticles[i].dPro = 0.0;
		// 			continue;
		// 		}


		if(m_pMap->get2DGridMap()[nX][nY] == CTWODMapState::UNKNOWN ||
			nX <= 0 || nX >= m_pMap->getMapSizeX() || nY <=0 || nY >= m_pMap->getMapSizeY() 			)
			m_pParticles[i].dPro = 0.0;

	}


}


void CRangeBasedPFLocalizer::update()
{

	vector<PolarRangeData>  * pPredictedPolarRangeData = new vector<PolarRangeData>  ;
	int nCnt = 0;	// the number of samples

	int nPre = 0; int nReal = 0;
	for(unsigned int i = 0 ; i < m_nParticleNum ; i++)
	{

		pPredictedPolarRangeData->clear();
		predictRangeData(  m_pParticles[i],  pPredictedPolarRangeData);
		nCnt = 0;
		m_pParticles[i].dUpdatePro = 0.0;	// Sample에 할당되는 확톩E

		for (int j=0;j<pPredictedPolarRangeData->size() ; j++ ) 
		{

// 			if (pPredictedPolarRangeData->at(j).r  == NULL || pPredictedPolarRangeData->at(j).r  == 0)
// 				continue;

			nPre = (int)(0.5 +pPredictedPolarRangeData->at(j).r *100.0);
			nReal = (int)(0.5 + m_PolarRangeData.at(j).r *100.0);

			if(nPre == 0) continue;
			if (nReal < 22 ) continue;
			if (nReal > m_Param.dRangeMaxDist * 100) continue;
			
			m_pParticles[i].dUpdatePro += (double)m_fRangeMCLPro[(int)(0.5 +pPredictedPolarRangeData->at(j).r *100.0)][(int)(0.5 + m_PolarRangeData.at(j).r *100.0)];
			nCnt++;
			//		m_stSample[i].dRangeUpdatePro *=pow((double)m_fRangeMCLPro[(int)(0.5 + m_dEstimatedRangeData[j]/10.0)][(int)(0.5 + m_dRangeData[j]/10.0)],0.02); // 0.2
		}

		if (nCnt>0)
		{
			m_pParticles[i].dUpdatePro /= (double)nCnt;
			//m_stSample[i].dMatchingError /= (double)nCnt;	//added CYK
			//m_dMinMatchingError += m_stSample[i].dMatchingError;
		}


		m_pParticles[i].dPro = m_pParticles[i].dPro * m_pParticles[i].dUpdatePro;
	}
	//cout << "pRangeDataSize : " << pPredictedPolarRangeData->size() << endl;
	delete pPredictedPolarRangeData;

}

void CRangeBasedPFLocalizer::resampling()
{
	struct Accumulation{	
		double dSum;
		double x;
		double y;
		double th;
	};
	Accumulation *pAccumulation;

	double dSum = 0.0;
	int nNoOfAccumulated;
	int nCnt;
	double dRandPro;
	int i;

	pAccumulation = NULL;
	pAccumulation = (Accumulation*)calloc(m_nParticleNum,sizeof(Accumulation));

	//TRACE("SD: %.3f\n", m_dSampleSD);
// 	if (m_dParticleSD<m_Param.nMinParticleSD)
// 	{
// 		CalculateNoOfSamples(false);
// 		cout<<"[CLaserBasedParticleFilter] : No sensor update\n"<<endl;
// 	}
// 	else
	{
		nNoOfAccumulated=0;
		for (i =0; i<m_nParticleNum; i++) 
		{
			dSum += m_pParticles[i].dPro;
			if (m_pParticles[i].dPro==0.0) continue;
			pAccumulation[nNoOfAccumulated].dSum = dSum;
			pAccumulation[nNoOfAccumulated].x = m_pParticles[i].x;
			pAccumulation[nNoOfAccumulated].y = m_pParticles[i].y;
			pAccumulation[nNoOfAccumulated].th = m_pParticles[i].th;
			nNoOfAccumulated++;
		}

		if(dSum == 0.0) 		
		{
			for (int i =0; i<m_nParticleNum; i++) 	
				m_pParticles[i].dPro =  1.0/(double)m_nParticleNum;
			return ;
		}
		CalculateNoOfSamples();

		int nTemp=0;

		while(nTemp<m_nParticleNum) {
			nCnt = 0;
			dRandPro = (double)rand()/(double)RAND_MAX;
			while (pAccumulation[nCnt].dSum<dRandPro && nCnt<nNoOfAccumulated-1){
				nCnt++;
			}
			m_pParticles[nTemp].x = pAccumulation[nCnt].x;
			m_pParticles[nTemp].y = pAccumulation[nCnt].y;
			m_pParticles[nTemp].th = pAccumulation[nCnt].th;
			m_pParticles[nTemp].dPro =  1.0/(double)m_nParticleNum;
			nTemp++;
		}
	}
	free (pAccumulation);
	pAccumulation = NULL;
}

void CRangeBasedPFLocalizer::Normalizing()
{
	double dSum = 0.0;
    int nCnt = 0;
    double dMax=0.0;

	for(int i=0; i<m_nParticleNum; i++)
		dSum += m_pParticles[i].dPro;
/*
	if (dSum==0.0) {
		for(int i=0; i<m_nSampleNum; i++)
			m_stSample[i].dPro = 1.0/(double)m_nSampleNum;
	} else {
		for(int i=0; i<m_nSampleNum; i++)
			m_stSample[i].dPro = m_stSample[i].dPro / dSum;
	}
*/
	if(dSum !=0.0){
		for(int i=0; i<m_nParticleNum; i++)
			m_pParticles[i].dPro = m_pParticles[i].dPro / dSum;
	}
}

CRangeBasedPFLocalizer::PolarRangeData CRangeBasedPFLocalizer::transRangeDataCartesianToPolar(CRobotState::Position RangePosition)
{
	PolarRangeData RangeData;
	double r = sqrt(  RangePosition.x*RangePosition.x + RangePosition.y*RangePosition.y  );
	if((RangePosition.x == 0.0 && RangePosition.y == 0.0) ||  r > m_Param.dRangeMaxDist)
	{
		RangeData.r = NULL;
		RangeData.th = NULL;
	}
	else
	{
		RangeData.r = r;
		RangeData.th = atan2(RangePosition.y , RangePosition.x );
		
	}
	return RangeData;
}



void CRangeBasedPFLocalizer::predictRangeData(Particle Sample, vector<PolarRangeData> * pPredictedPolarRangeData  )
{
	double x = Sample.x; double y = Sample.y; double t = Sample.th;

	int nPosX1=0,nPosY1=0;
	int nPosX2=0,nPosY2=0;
	int nHitX=0, nHitY=0;	// Ray-tracing을 흟E?얻은 지도에서의 샘플 각각의 거리정보

	double dAngle=0.;
	double dPosTh=0.;

	double dCellSize = m_pMap->getCellSize();

	// 100으로 나눠주는 이유는 10cm격자에 맞춰서 방향을 이동시키콅E위함이다.
	nPosX1 = (int)(x  /dCellSize);		// 샘플의 위치에서 레이픸E센서 방향에 따른 X퉩EE방향벡터
	nPosY1 = (int)(y  / dCellSize);			// 샘플의 위치에서 레이픸E센서 방향에 따른 Y퉩EE방향벡터


	//광선하나하나
	PolarRangeData PredictedData; PredictedData.r = 0.0; PredictedData.th = 0.0;
	for(unsigned int i = 0 ; i < m_PolarRangeData.size() ; i++)
	{

		nPosX2 = nPosX1 + (int)(m_Param.dRangeMaxDist*cos(Sample.th+m_PolarRangeData[i].th)/dCellSize);	// 센서 감햨E거리에 따른 X방향의 최큱E감햨E퉩E?설정.	
		nPosY2 = nPosY1 + (int)(m_Param.dRangeMaxDist*sin(Sample.th+m_PolarRangeData[i].th)/dCellSize);	// 센서 감햨E거리에 따른 Y방향의 최큱E감햨E퉩E?설정.


		doRayTracing(nPosX1, nPosX2, nPosY1, nPosY2, &nHitX, &nHitY); // Ray-tracing 펯EE
		// 지도의 경컖E근처에 있는 경퓖E?따지는 부분
		if (nHitX<=10 || nHitX >= (m_pMap->getMapSizeX()-10) || nHitY<=10 || nHitY >=(m_pMap->getMapSizeY()-10)) 
		{
			PredictedData.r = NULL;
			PredictedData.th = NULL;
			
		}
// 		else if (m_PolarRangeData[i].th == NULL && m_PolarRangeData[i].r == NULL)
// 		{
// 			PredictedData.r = NULL;
// 			PredictedData.th = NULL;
// 		}
		else
		{
			PredictedData.r = sqrt((double)(nHitX*dCellSize-nPosX1*dCellSize)*(nHitX*dCellSize-nPosX1*dCellSize)+
				(double)(nHitY*dCellSize-nPosY1*dCellSize)*(nHitY*dCellSize-nPosY1*dCellSize )	);
			PredictedData.th = m_PolarRangeData[i].th;

		}
		pPredictedPolarRangeData->push_back(PredictedData);
	}
}


/**
 @brief : Function executing ray-tracing with vector of samples' position
*/
void CRangeBasedPFLocalizer::doRayTracing(int x_1, int x_2, int y_1, int y_2, int *hit_x, int *hit_y)
{

	int eps = 0;
	int x_thres=x_2, y_thres=y_2;
/////////////////////////////////////////////////////////////////////
	if(x_1 < 0) x_1 = 0;
	else if(x_1 >= m_pMap->getMapSizeX()) x_1 = m_pMap->getMapSizeX()-1;
	if(y_1 < 0) y_1 = 0;
	else if(y_1 >= m_pMap->getMapSizeY()) y_1 = m_pMap->getMapSizeY()-1;

	//이것을 주석 해제하툈E자도 경계부근에서 레이트래이싱결과가 이상해햨E
	if(x_thres < 0) x_thres = 0;
	else if(x_thres >= m_pMap->getMapSizeX()) x_thres = m_pMap->getMapSizeX()-1;
	if(y_thres < 0) y_thres = 0;
	else if(y_thres >= m_pMap->getMapSizeY()) y_thres = m_pMap->getMapSizeY()-1;
/////////////////////////////////////////////////////////////////////
		int x = x_1,y = y_1;

		int delx = x_thres - x_1;
		int dely = y_thres - y_1;


	if(delx > 0) {
		if(dely >=0) {
			if(delx > dely) {
				for(int x=x_1; x<=x_thres; x++) {
  					if(m_pMap->get2DGridMap()[x][y] == CTWODMapState::UNKNOWN || m_pMap->get2DGridMap()[x][y] == CTWODMapState::OCCUPIED ) {
    					// 거리값을 넣는다.
						*hit_x = x;
					    	*hit_y = y;
    						return;
  					}

  					eps += dely;
  					if((eps<<1) >= delx) {
  						y++;
    					eps -= delx;
  					}
				}

			}
			else { //delx <= dely인 경퓖E

				for(y=y_1; y<=y_thres; y++) {
					if(m_pMap->get2DGridMap()[x][y] == CTWODMapState::UNKNOWN  || m_pMap->get2DGridMap()[x][y] == CTWODMapState::OCCUPIED ) {
    					// 거리값을 넣는다.
						*hit_x = x;
						*hit_y = y;
						return;
  					}

  					eps += delx;
  					if((eps<<1) >= dely) {
    						x++;
    						eps -= dely;
  					}
				}

			}

		}
		else { // dely < 0인경유
			if(delx > -dely) {
				for(x=x_1; x<=x_thres; x++) {
  					if(m_pMap->get2DGridMap()[x][y] ==CTWODMapState::UNKNOWN  || m_pMap->get2DGridMap()[x][y] == CTWODMapState::OCCUPIED ) {
    					// 거리값을 넣는다.
					       *hit_x = x;
					       *hit_y = y;
    					       return;
  					}

					eps += dely;
					if((eps<<1) <= -delx) {
						y--;
						eps += delx;
  					}
				}

			}
			else { //delx <= -dely인 경퓖E

				for(y=y_1; y>=y_thres; y--) {
  					if(m_pMap->get2DGridMap()[x][y] == CTWODMapState::UNKNOWN  || m_pMap->get2DGridMap()[x][y] == CTWODMapState::OCCUPIED ) {
    					// 거리값을 넣는다.
						*hit_x = x;
						*hit_y = y;
						return;
					}

  					eps += delx;
  					if((eps<<1) >= -dely) {
    						x++;
    						eps += dely;
  					}
				}

			}
		}
	}

	else { //delx <=0인경퓖E
		if(dely >= 0) {
			if(-delx > dely) {

				for(x=x_1; x>=x_thres; x--) {
					if(m_pMap->get2DGridMap()[x][y] == CTWODMapState::UNKNOWN || m_pMap->get2DGridMap()[x][y] == CTWODMapState::OCCUPIED ) {
					// 거리값을 넣는다.
						*hit_x = x;
						*hit_y = y;
						return;
					}

					eps += dely;
					if((eps<<1) >= -delx) {
						y++;
						eps += delx;
					}
				}

			}
			else { //-delx <= dely인경퓖E

				for(y=y_1; y<=y_thres; y++) {
					if(m_pMap->get2DGridMap()[x][y] == CTWODMapState::UNKNOWN  || m_pMap->get2DGridMap()[x][y] == CTWODMapState::OCCUPIED ) {
					// 거리값을 넣는다.
						*hit_x = x;
						*hit_y = y;
						return;
					}

					eps += delx;
					if((eps<<1) <= -dely) {
						x--;
						eps += dely;
					}
				}

			}
		}
		else { //dely < 0인경퓖E
			if(-delx > -dely) {
				for(x=x_1; x>=x_thres; x--) {
					if(m_pMap->get2DGridMap()[x][y] == CTWODMapState::UNKNOWN  || m_pMap->get2DGridMap()[x][y] == CTWODMapState::OCCUPIED ) {
						// 거리값을 넣는다.
							*hit_x = x;
							*hit_y = y;
							return;
					}

					eps -= dely;
					if((eps<<1) > -delx) {
						y--;
						eps += delx;
					}
				}

			}
			else { //-delx <= -dely인경퓖E
				for(y=y_1; y>=y_thres; y--) {
					if(m_pMap->get2DGridMap()[x][y] == CTWODMapState::UNKNOWN  || m_pMap->get2DGridMap()[x][y] == CTWODMapState::OCCUPIED ) {
					// 거리값을 넣는다.
						*hit_x = x;
						*hit_y = y;
						return;
					}

					eps -= delx;
					if((eps<<1) >= -dely) {
						x--;
						eps += dely;
					}
				}

			}
		}
	}

}


 //m_pMap->get2DGridMap()[nStartPosition[0]][nStartPosition[1]] == CTWODMapState::OCCUPIED


void CRangeBasedPFLocalizer::CalculateNoOfSamples()
{
	double dAvgPose[3], dSD[3];		// 샘플의 평균 및 표준폴邪를 구하콅E위한 변펯E


		// 확톩E?과潭하여 분퍊E갋다음 단컖E샘플의 갯수를 계퍊E?
		dAvgPose[0] = 0.0;
		dAvgPose[1] = 0.0;
		dAvgPose[2] = 0.0;
		dSD[0] = 0.0;
		dSD[1] = 0.0;
		dSD[2] = 0.0;

		for(int i = 0; i<m_nParticleNum; i++) {
			dAvgPose[0] += m_pParticles[i].x*m_pParticles[i].dPro;
			dAvgPose[1] += m_pParticles[i].y*m_pParticles[i].dPro;
			dAvgPose[2] += m_pParticles[i].th*m_pParticles[i].dPro;
		}
		for(int i = 0; i<m_nParticleNum; i++) {
			dSD[0] += (m_pParticles[i].x-dAvgPose[0])*(m_pParticles[i].x-dAvgPose[0])*m_pParticles[i].dPro;
			dSD[1] += (m_pParticles[i].y-dAvgPose[1])*(m_pParticles[i].y-dAvgPose[1])*m_pParticles[i].dPro;
			dSD[2] += (m_pParticles[i].th-dAvgPose[2])*(m_pParticles[i].th-dAvgPose[2])*m_pParticles[i].dPro;
		}
 		m_dParticleSD = sqrt(dSD[0] + dSD[1]);
// 		m_dSampleSDForRotation = sqrt(dSD[2]);

		dSD[0] = sqrt(dSD[0]);
		dSD[1] = sqrt(dSD[1]);

		// 샘플의 표준폴邪를 이퓖E臼?다음 단계에 추출될 샘플의 갯수를 계퍊E
		m_nParticleNum = (int)( 1.0*(dSD[0]*dSD[1])*m_Param.nParticleDensity );

		// 최큱E?이퍊E 최소값 이하의 샘플 갯수가 계퍊E퓔갋최큱E? 최소값으로 큱E?
		if (m_nParticleNum > m_Param.nMaxParticleNum) m_nParticleNum = m_Param.nMaxParticleNum;
		if (m_nParticleNum < m_Param.nMinParticleNum) m_nParticleNum = m_Param.nMinParticleNum;
	
}

void CRangeBasedPFLocalizer::estimation()
{
	double dSum = 0.0;
	// 	if (m_dSamplePos[2]>120*D2R || m_dSamplePos[2]<-120*D2R) bAngleCalculationMode = true;
	// 	else bAngleCalculationMode = false;
	double dWSumX = 0.0;
	double dWSumY = 0.0;
	double dWSumZ = 0.0;
	// calculating robot pose by averaging poses of samples
	double dx, dy;
	dx = 0; dy = 0;
	for(int i=0; i<m_nParticleNum; i++)  
	{
		dSum += m_pParticles[i].dPro;
		dWSumX += m_pParticles[i].x*m_pParticles[i].dPro;
		dWSumY += m_pParticles[i].y*m_pParticles[i].dPro;
		
		//if(bAngleCalculationMode && m_stSample[i].t<0) m_dSamplePos[2] += (m_stSample[i].t+2.0*M_PI)*m_stSample[i].dPro;
		//else m_dSamplePos[2] += m_stSample[i].t*m_stSample[i].dPro;
		dx += cos(m_pParticles[i].th)*m_pParticles[i].dPro;
		dy += sin(m_pParticles[i].th)*m_pParticles[i].dPro;
	}
	double dCov[3][3] = {0};
	double xe = 0.0;
	double ye = 0.0;
	double te = 0.0;

	for(int i=0; i<m_nParticleNum; i++)  
	{
		xe = (m_pParticles[i].x - dWSumX/dSum);
		ye =  (    m_pParticles[i].y - dWSumY/dSum);
		te =  (     atan2(sin(m_pParticles[i].th), cos(m_pParticles[i].th)) - atan2(dy/dSum, dx/dSum) );
	
		 dCov[0][0] += xe*xe;   dCov[0][1] += xe*ye;   dCov[0][2] += xe*te;
		 dCov[1][0] += xe*ye;   dCov[1][1] += ye*ye;   dCov[1][2] += ye*te;
		 dCov[2][0] += te*xe;   dCov[2][1] += te*ye;   dCov[2][2] += te*te;	
	}

	if (dSum!=0.0) 
	{
		
		m_RobotState.setX(dWSumX/dSum);
		m_RobotState.setY(dWSumY/dSum);
		m_RobotState.setYaw(atan2(dy/dSum, dx/dSum)   );

		m_RobotState.setCovarianceElement(0,0,   dCov[0][0]  / (m_nParticleNum-1));
		m_RobotState.setCovarianceElement(0,1,   dCov[0][1]  / (m_nParticleNum-1));
		m_RobotState.setCovarianceElement(0,2,   dCov[0][2]  / (m_nParticleNum-1));

		m_RobotState.setCovarianceElement(1,0,   dCov[1][0]  / (m_nParticleNum-1));
		m_RobotState.setCovarianceElement(1,1,   dCov[1][1]  / (m_nParticleNum-1));
		m_RobotState.setCovarianceElement(1,2,   dCov[1][2]  / (m_nParticleNum-1));

		m_RobotState.setCovarianceElement(2,0,   dCov[2][0]  / (m_nParticleNum-1));
		m_RobotState.setCovarianceElement(2,1,   dCov[2][1]  / (m_nParticleNum-1));
		m_RobotState.setCovarianceElement(2,2,   dCov[2][2]  / (m_nParticleNum-1));
	}
}



void CRangeBasedPFLocalizer::initParticles(double x, double y, double th, double dRadiusScope, double dAngleScope)
{
	m_nParticleNum = m_Param.nMaxParticleNum;
	m_pParticles = new Particle[m_nParticleNum];

	int nCnt = 0;
	double dx, dy, dr;

	m_dParticleSD = m_Param.nMinParticleSD;

	// 샘플이 뿌려햨E영역 및 SAMPLE_DENSITY값에 따른 샘플 갯펯E계퍊E
	m_nParticleNum = (int)(dRadiusScope*dRadiusScope*4  *m_Param.nParticleDensity);
	//cout << "Particle Num : " << m_nParticleNum << endl;
	if (m_nParticleNum > m_Param.nMaxParticleNum) m_nParticleNum = m_Param.nMaxParticleNum;
	if (m_nParticleNum < m_Param.nMinParticleNum) m_nParticleNum = m_Param.nMinParticleNum;

	random_device rd;
	mt19937 rEngine(rd());
	uniform_int_distribution<> dist(0, dRadiusScope*1000.0*2);

	random_device rdth;
	mt19937 rEngineth(rdth());
	uniform_int_distribution<> distth(0, dAngleScope*1000.0*2);

	int nGX, nGY; int nmRad;
	int nX = 0; int nY = 0;
	while(nCnt<m_nParticleNum)
	{
		nGX = dist(rEngine);
		nGY = dist(rEngine); //mm
		nmRad = distth(rEngineth);

		m_pParticles[nCnt].x = (x - dRadiusScope)  +  (double)nGX*0.001;
		m_pParticles[nCnt].y = (y - dRadiusScope)  +  (double)nGY*0.001;
		m_pParticles[nCnt].z = 0.0; 
		m_pParticles[nCnt].th = (th - dAngleScope) + (double)nmRad*0.001;
		if (m_pParticles[nCnt].th>M_PI) m_pParticles[nCnt].th -=2*M_PI;
		else if (m_pParticles[nCnt].th<-M_PI) m_pParticles[nCnt].th +=2*M_PI;
		m_pParticles[nCnt].dPro = 1/(double)m_nParticleNum;

		nX = (int)(m_pParticles[nCnt].x/m_pMap->getCellSize());
		nY = (int)(m_pParticles[nCnt].y/m_pMap->getCellSize());

		if( nX <= 0 || nX >= m_pMap->getMapSizeX() || nY <=0 || nY >= m_pMap->getMapSizeY()	)
			continue;

		if(m_pMap->get2DGridMap()[nX][nY] == CTWODMapState::UNKNOWN
			|| m_pMap->get2DGridMap()[nX][nY] == CTWODMapState::OCCUPIED ) continue;

		//cout << m_pParticles[nCnt].x  << " " << m_pParticles[nCnt].y  << " " <<  m_pParticles[nCnt].th << endl;
		nCnt++;
	}
}

void CRangeBasedPFLocalizer::setRangeData(vector<CRobotState::Position> RangeData) 
{
	m_PolarRangeData.clear();
	
	for(int i = 0 ; i < RangeData.size() ; i = i + m_Param.nRangeInterval )
	{
		double r = sqrt(  RangeData.at(i).x*RangeData.at(i).x + RangeData.at(i).y*RangeData.at(i).y  );
		//if (r > 0.22 && r < m_Param.dRangeMaxDist - 0.001)
			m_PolarRangeData.push_back(transRangeDataCartesianToPolar(RangeData.at(i)));
	}

}

bool CRangeBasedPFLocalizer::isTimeToUpdate()
{
	if(m_PolarRangeData.size() == 0)
		return false;

// 	double dSum = 0.0;
// 	for (int i =0; i<m_nParticleNum; i++) 		dSum += m_pParticles[i].dPro;
// 	if(dSum == 0.0) 		return false;

	//if (m_AccumulatedMovement.r > 0.3 || m_AccumulatedMovement.th > 10.0*D2R) 
	//if (m_AccumulatedMovement.r > 0.5 || m_AccumulatedMovement.th > 20.0*D2R) 
	if (m_AccumulatedMovement.r > 0.15 || m_AccumulatedMovement.th > 5.0*D2R)  //For Mapping?
	{
			m_AccumulatedMovement.r = 0.0;
			m_AccumulatedMovement.th = 0.0;
			return true;
	}
		
	return false;
}