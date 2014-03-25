#include "CCartesianCloud.h"
#include <iostream>

namespace fair
{

CCartesianCloud3D::CCartesianCloud3D()
{
	_nHasInfo = 0;

	_adTranslation[0] = 0.0;
	_adTranslation[1] = 0.0;
	_adTranslation[2] = 0.0;
	_rotPoint.dX = 0.0;
	_rotPoint.dY = 0.0;
	_rotPoint.dZ = 0.0;
	_mRotationMatrix = IdentityMatrix44();
	_vPoints.clear();
	_vInfo.clear();
	
	_bForceUserDefined = false;
	_fPointSize = 0.0f;
}

CCartesianCloud3D::~CCartesianCloud3D()
{
	this->erase();
	_mSourceInfo.clear();
}

CCartesianCloud3D* CCartesianCloud3D::create(unsigned int unPoints, bool bHasInfo)
{
	CCartesianCloud3D* cloud = new CCartesianCloud3D();
	
	if(bHasInfo)
	{
		for(unsigned int i = 0; i < unPoints; i++)
		{
			fair::StrCartesianPoint3D* point = new fair::StrCartesianPoint3D();
			fair::StrPointInfo* info = new fair::StrPointInfo();
			info->afRGB[0] = 255.0;
			info->afRGB[1] = 255.0;
			info->afRGB[2] = 255.0;
			info->bValid = true;
			cloud->add(point,info);
		}
	}
	else
	{
		for(unsigned int i = 0; i < unPoints; i++)
			cloud->add(new fair::StrCartesianPoint3D());
	}
	
	return cloud;
}

CCartesianCloud3D* CCartesianCloud3D::createSubCloud(vector<unsigned int>* pvIdx)
{
	unsigned int unSize = pvIdx->size();
	CCartesianCloud3D* cloudNew = new CCartesianCloud3D();
	if(_nHasInfo)
	{
		for(unsigned int i=0; i<unSize; i++)
		{
			unsigned int unIdx = (*pvIdx)[i];
			fair::StrCartesianPoint3D* point = _vPoints[unIdx];
			fair::StrPointInfo* info = _vInfo[unIdx];
			fair::StrCartesianPoint3D* pointNew = new StrCartesianPoint3D(point);
			fair::StrPointInfo* infoNew = new StrPointInfo(info);
			cloudNew->add(pointNew, infoNew);
		}
	}
	else
	{
		for(unsigned int i=0; i<unSize; i++)
		{
			unsigned int unIdx = (*pvIdx)[i];
			fair::StrCartesianPoint3D* point = _vPoints[unIdx];
			fair::StrCartesianPoint3D* pointNew = new StrCartesianPoint3D(point);
			cloudNew->add(pointNew);
		}
	}
	return cloudNew;
}

StrCartesianPoint3D* CCartesianCloud3D::operator [] (unsigned int i) {
	fairAssert(_vPoints.size() > i, "Access to invalid element of point cloud");
    return _vPoints[i];
}

StrCartesianPoint3D* CCartesianCloud3D::getPoint(unsigned int i)
{
	fairAssert(_vPoints.size() > i, "Access to invalid element of point cloud");
    return _vPoints[i];
}
	
StrPointInfo* CCartesianCloud3D::getInfo(unsigned int i)
{
	StrPointInfo* info = NULL;
	if(_vInfo.size() > i) info = _vInfo[i];
    return info;
}

void CCartesianCloud3D::getRawCoordinates(double* pdBuffer)
{
	unsigned int unIdx = 0;
	for(unsigned int i=0; i<_vPoints.size(); i++)
	{
		StrCartesianPoint3D* point = _vPoints[i];
		pdBuffer[unIdx++] = point->dX;
		pdBuffer[unIdx++] = point->dY;
		pdBuffer[unIdx++] = point->dZ;
	}
}

void CCartesianCloud3D::getRawCoordinatesInt(double* pdBuffer)
{
	if(this->hasInfo() == false)
	{
		fair::fatal("CCartesianCloud3D:getRawCoordinatesInt Cloud contains no point information");	
	}
	unsigned int unIdx = 0;
	for(unsigned int i=0; i<_vPoints.size(); i++)
	{
		StrCartesianPoint3D* point = _vPoints[i];
		StrPointInfo* info = _vInfo[i];
		
		if ( !info->bValid )
		    continue;
		
		pdBuffer[unIdx++] = point->dX;
		pdBuffer[unIdx++] = point->dY;
		pdBuffer[unIdx++] = point->dZ;
		pdBuffer[unIdx++] = info->fIntensity;
	}
}

void CCartesianCloud3D::getRawCoordinatesUserData(double* pdBuffer)
{
	if(this->hasInfo() == false)
	{
		fair::fatal("CCartesianCloud3D:getRawCoordinatesInt Cloud contains no point information");	
	}
	unsigned int unIdx = 0;
	for(unsigned int i=0; i<_vPoints.size(); i++)
	{
		StrCartesianPoint3D* point = _vPoints[i];
		StrPointInfo* info = _vInfo[i];
		pdBuffer[unIdx++] = point->dX;
		pdBuffer[unIdx++] = point->dY;
		pdBuffer[unIdx++] = point->dZ;
		double* pdUserData = info->pdUserData;
		for(unsigned int j=0; j<info->unUserDataLength; j++)
			pdBuffer[unIdx++] = pdUserData[j];
	}	
}

void CCartesianCloud3D::copy(CCartesianCloud3D* cloud)
{
	fairAssert(cloud->size()==_vPoints.size(),"Copy operation for clouds of different size not possible");
	for(unsigned int i=0; i<_vPoints.size(); i++)
	{
		StrCartesianPoint3D* point = _vPoints[i];
		point->dX = (*cloud)[i]->dX;
		point->dY = (*cloud)[i]->dY;
		point->dZ = (*cloud)[i]->dZ;
		if(cloud->hasInfo() && this->hasInfo())
		{
			StrPointInfo* infoDst = _vInfo[i];
			StrPointInfo* infoSrc = cloud->getInfo(i);
			infoDst->fIntensity = infoSrc->fIntensity;
			infoDst->fAmplitude = infoSrc->fAmplitude;
			infoDst->dAccuracy = infoSrc->dAccuracy;
			infoDst->afRGB[0] = infoSrc->afRGB[0];
			infoDst->afRGB[1] = infoSrc->afRGB[1];
			infoDst->afRGB[2] = infoSrc->afRGB[2];
			infoDst->bValid = infoSrc->bValid;	
		}
	}
}

void CCartesianCloud3D::getCartesianCoordinates(double** ppdCloud)
{
    fairAssert(_vPoints.size()>0,"Get operation on empty cloud");
    for(unsigned int i=0; i<_vPoints.size(); i++)
    {
        double* pdPoint = ppdCloud[i];
        StrCartesianPoint3D* pointSrc = _vPoints[i];
        pdPoint[0] = pointSrc->dX;
        pdPoint[1] = pointSrc->dY;
        pdPoint[2] = pointSrc->dZ;
    }   
}

unsigned int CCartesianCloud3D::getCartesianCoordinatesRH(double** ppdCloud)
{
	unsigned int unRetVal = 0;
	fairAssert(_vPoints.size()>0,"Get operation on empty cloud");
	for(unsigned int i=0; i<_vPoints.size(); i++)
	{
		if ( this->hasInfo() && !this->getInfo(i)->bValid )
			continue;
		ppdCloud[unRetVal][0] = -_vPoints[i]->dZ;
		ppdCloud[unRetVal][1] = -_vPoints[i]->dX;
		ppdCloud[unRetVal][2] = _vPoints[i]->dY;
		++unRetVal;
	}
	return unRetVal;
}

void CCartesianCloud3D::add(StrCartesianPoint3D* point) {
	_vPoints.push_back(point);
	_nHasInfo = 0;
}

void CCartesianCloud3D::add(StrCartesianPoint3D* point, StrPointInfo* info) {
	fairAssert(_vPoints.size() == _vInfo.size(), "Point cloud and point info have different sizes");
	_nHasInfo = 1;
	_vPoints.push_back(point);
	_vInfo.push_back(info);
}

void CCartesianCloud3D::add(CCartesianCloud3D* cloud, bool bCheckValidFlag)
{
	fairAssert(cloud->hasInfo() == hasInfo(), "CCartesianCloud3D::add: Source and target cloud should have both either info or not!");
	unsigned int unSize = cloud->size();
	if(cloud->hasInfo())
	{
		if(bCheckValidFlag)
		{
			for(unsigned int i=0; i<unSize; i++)
			{
				StrCartesianPoint3D* point = new StrCartesianPoint3D((*cloud)[i]);
				StrPointInfo* info = new StrPointInfo(cloud->getInfo(i));
				if(info->bValid) add(point, info);
			}
		}
		else
		{
			for(unsigned int i=0; i<unSize; i++)
			{
				StrCartesianPoint3D* point = new StrCartesianPoint3D((*cloud)[i]);
				StrPointInfo* info = new StrPointInfo(cloud->getInfo(i));
				add(point, info);		
			}
		}
	}
	else
	{
		for(unsigned int i=0; i<unSize; i++)
		{
			StrCartesianPoint3D* point = new StrCartesianPoint3D((*cloud)[i]);
			add(point);		
		}
	}
}

void CCartesianCloud3D::add(CCartesianCloud3D* cloud, vector<unsigned int>* vIdx)
{
	fairAssert(cloud->hasInfo() == hasInfo(), "CCartesianCloud3D::add: Source and target cloud should have both either info or not!");
	unsigned int unSize = vIdx->size();
	if(cloud->hasInfo())
	{
		for(unsigned int i=0; i<unSize; i++)
		{
			unsigned int unIdx = (*vIdx)[i];
			StrCartesianPoint3D* point = new StrCartesianPoint3D((*cloud)[unIdx]);
			StrPointInfo* info = new StrPointInfo(cloud->getInfo(unIdx));
			add(point, info);		
		}
	}
	else
	{
		for(unsigned int i=0; i<unSize; i++)
		{
			unsigned int unIdx = (*vIdx)[i];
			StrCartesianPoint3D* point = new StrCartesianPoint3D((*cloud)[unIdx]);
			add(point);		
		}
	}
}

int CCartesianCloud3D::hasInfo()
{
	return _nHasInfo;	
}

int CCartesianCloud3D::hasSourceInfo()
{
	return (!_mSourceInfo.empty());
}

void CCartesianCloud3D::addSourceInfo(EnumSourceInfo eSourceInfo, long lValue)
{
	_mSourceInfo[eSourceInfo] = lValue;
}
  	
int CCartesianCloud3D::getSourceInfo(EnumSourceInfo eSourceInfo, long* plValue)
{
	int nRetval = 0;
	if (_mSourceInfo.find(eSourceInfo) != _mSourceInfo.end())
	{
		nRetval = 1;
		*plValue = _mSourceInfo[eSourceInfo];
	}
	return nRetval;
}

int CCartesianCloud3D::removeSourceInfo(EnumSourceInfo eSourceInfo)
{
	int nRetval = 0;
	if (_mSourceInfo.find(eSourceInfo) != _mSourceInfo.end())
	{
		nRetval = 1;
		_mSourceInfo.erase(eSourceInfo);
	}
	return nRetval;	
}

void CCartesianCloud3D::removeInvalidPoints()
{
	if(_nHasInfo == 0) return;
	
	vector<StrCartesianPoint3D*> vPointsTmp;
	vector<StrPointInfo*> vInfoTmp;
	
	unsigned int unSize = _vPoints.size();
	unsigned int i = 0;
	for(i=0; i<unSize; i++)
	{
		StrPointInfo* info = _vInfo[i];
		if(info->bValid == false)
		{
			StrCartesianPoint3D* point = _vPoints[i];
			delete point;
			delete info;
		}
		else
		{
			vPointsTmp.push_back(_vPoints[i]);
			vInfoTmp.push_back(info);	
		}
	}
	_vPoints.clear();
	_vInfo.clear();
	
	unSize = vPointsTmp.size();
	for(i=0; i<unSize; i++)
	{
		_vPoints.push_back(vPointsTmp[i]);
		_vInfo.push_back(vInfoTmp[i]);	
	}
}

void CCartesianCloud3D::clearSourceInfo()
{
	_mSourceInfo.clear();	
}

unsigned int CCartesianCloud3D::size()
{
	return _vPoints.size();
}

void CCartesianCloud3D::erase()
{
	unsigned int i = 0;
		
	for(i=0; i<_vPoints.size(); i++)
		delete(_vPoints[i]);
	for(i=0; i<_vInfo.size(); i++)
	{
		double* pdUserData = (*_vInfo[i]).pdUserData;
		if ( pdUserData != NULL )
		{
			cout << "aua " << i << endl;
			delete [] pdUserData;
		}
		delete(_vInfo[i]);
	}

	_vPoints.clear();
	_vInfo.clear();
}

void CCartesianCloud3D::clear()
{
	_vPoints.clear();
	_vInfo.clear();
}

void CCartesianCloud3D::serialize(IOutputStream* stream)
{
	// Number of points
	stream->write((int)_vPoints.size());

	// Point info provided
	stream->write(hasInfo());

	// Source info
	long lValue = 0;
	getSourceInfo(eSOURCEROWS,&lValue);
	stream->write((long)eSOURCEROWS);
	stream->write(lValue);
	lValue = 0;
	getSourceInfo(eSOURCECOLS,&lValue);
	stream->write((long)eSOURCECOLS);
	stream->write(lValue);
	// Data
	for(unsigned int i=0; i<_vPoints.size(); i++)
	{
		stream->write((*_vPoints[i]).dX);
		stream->write((*_vPoints[i]).dY);
		stream->write((*_vPoints[i]).dZ);
		if(hasInfo())
		{
			StrPointInfo* info = getInfo(i);
			stream->write(info->fIntensity);
			stream->write(info->fAmplitude);
			stream->write(info->dAccuracy);
			stream->write(info->afRGB[0]);
			stream->write(info->afRGB[1]);
			stream->write(info->afRGB[2]);
			stream->write((int)info->bValid);
		}
		
	}
}

CCartesianCloud3D* CCartesianCloud3D::load(IInputStream* stream)
{
	// Input Buffer
	int nVal;
	long lVal;
	float fVal;
	double dVal;
	
	// Number of points
	int nPointsCnt = 0;
	stream->read(nPointsCnt);

	// Point info provided
	int nHasInfo = 0;
	stream->read(nHasInfo);

	CCartesianCloud3D* cloud = CCartesianCloud3D::create((unsigned int)nPointsCnt, nHasInfo!=0);

	// Source info
	long lInfoID;
	stream->read(lInfoID);
	stream->read(lVal);
	if(lVal!=0) cloud->addSourceInfo(static_cast<EnumSourceInfo>(lInfoID),lVal);
	stream->read(lInfoID);
	stream->read(lVal);	
	if(lVal!=0) cloud->addSourceInfo(static_cast<EnumSourceInfo>(lInfoID),lVal);
	
	// Data
	for(int i=0; i<nPointsCnt; i++)
	{
		StrCartesianPoint3D* point = (*cloud)[i];
		stream->read(dVal);
		point->dX = dVal;
		stream->read(dVal);
		point->dY = dVal;
		stream->read(dVal);
		point->dZ = dVal;
		if(nHasInfo)
		{
			StrPointInfo* info = cloud->getInfo(i);
			stream->read(fVal);
			info->fIntensity = fVal;
			stream->read(fVal);
			info->fAmplitude = fVal;
			stream->read(dVal);
			info->dAccuracy = dVal;
			stream->read(fVal);
			info->afRGB[0] = fVal;
			stream->read(fVal);
			info->afRGB[1] = fVal;
			stream->read(fVal);
			info->afRGB[2] = fVal;
			stream->read(nVal);
			info->bValid = (nVal!=0);
		}
	}
	return cloud;
}

void CCartesianCloud3D::setForceDisplayUserDefined(bool bForce)
{
	_bForceUserDefined = bForce;
}

bool CCartesianCloud3D::getForceDisplayUserDefined()
{
	return _bForceUserDefined;	
}

void CCartesianCloud3D::setPointSize(float fPointSize)
{
	_fPointSize = fPointSize;	
}

float CCartesianCloud3D::getPointSize()
{
	return _fPointSize;	
}

void CCartesianCloud3D::setColor(float* pfColor)
{
	if(_nHasInfo)
	{
		for(unsigned int i=0; i<_vInfo.size(); i++)
		{
			StrPointInfo* info = _vInfo[i];
			info->afRGB[0] = pfColor[0];
			info->afRGB[1] = pfColor[1];
			info->afRGB[2] = pfColor[2];
		}	
	}
}

void CCartesianCloud3D::dump()
{
	// Number of points
	cout << "Size: " << _vPoints.size() << endl;

	// Point info provided
	cout << "HasInfo: " << hasInfo() << endl;

	// Source info
	long lValue = 0;
	getSourceInfo(eSOURCEROWS,&lValue);
	cout << "Source Rows: " << lValue << endl;
	lValue = 0;
	getSourceInfo(eSOURCECOLS,&lValue);
	cout << "Source Cols: " << lValue << endl;
	cout << "X | Y | Z";
	if(hasInfo()) cout << " | I | A | Acc | R | G | B | V";
	cout << endl;
	
	// Data
	for(unsigned int i=0; i<_vPoints.size(); i++)
	{
		cout << (*_vPoints[i]).dX << " ";
		cout << (*_vPoints[i]).dY << " ";
		cout << (*_vPoints[i]).dZ;
		if(hasInfo())
		{
			StrPointInfo* info = getInfo(i);
			cout << " " << info->fIntensity << " ";
			cout << info->fAmplitude << " ";
			cout << info->dAccuracy << " ";
			cout << info->afRGB[0] << " ";
			cout << info->afRGB[1] << " ";
			cout << info->afRGB[2] << " ";
			cout << (int)info->bValid;
		}
		cout << endl;
		
	}
}

StrCartesianPoint3D* CCartesianCloud3D::getCentroid()
{
	double dXMean = 0;
	double dYMean = 0;
	double dZMean = 0;
	StrCartesianPoint3D* pointDst = new StrCartesianPoint3D();
	pointDst->dX = 0;
	pointDst->dY = 0;
	pointDst->dZ = 0;
	if(_vPoints.size()==0) return pointDst;
	for(unsigned int i = 0; i<_vPoints.size(); i++)
	{
		StrCartesianPoint3D* pointSrc = _vPoints[i];
		dXMean += pointSrc->dX;
		dYMean += pointSrc->dY;
		dZMean += pointSrc->dZ;
	}
	double dSizeInv = 1.0 / (double)_vPoints.size();
	pointDst->dX = dXMean * dSizeInv;
	pointDst->dY = dYMean * dSizeInv;
	pointDst->dZ = dZMean * dSizeInv;
	return pointDst;
}

StrCartesianPoint3D* CCartesianCloud3D::getCentroidValidPoints()
{
	double dXMean = 0;
	double dYMean = 0;
	double dZMean = 0;
	unsigned int unCount = 0;
	StrCartesianPoint3D* point = new StrCartesianPoint3D();
	point->dX = 0;
	point->dY = 0;
	point->dZ = 0;
	if(_vPoints.size()==0) return point;
	if(this->hasInfo()==false) return point;
	for(unsigned int i = 0; i<_vPoints.size(); i++)
	{
		if(getInfo(i)->bValid==true)
		{
			StrCartesianPoint3D* pointSrc = _vPoints[i];
			dXMean += pointSrc->dX;
			dYMean += pointSrc->dY;
			dZMean += pointSrc->dZ;
			unCount++;
		}
	}
	if(unCount>0)
	{
		double dCount =  1.0 / (double) unCount;
		point->dX = dXMean * dCount;
		point->dY = dYMean * dCount;
		point->dZ = dZMean * dCount;
	}
	return point;
}

void CCartesianCloud3D::transform(CMatrix44* matrix)
{
	double dXTemp;
	double dYTemp;
	double dZTemp;
	for(unsigned int i = 0; i<_vPoints.size(); i++)
	{
		StrCartesianPoint3D* point = _vPoints[i];
		double dX = point->dX;
		double dY = point->dY;
		double dZ = point->dZ;
		
		dXTemp = 		dX 	* (*matrix)[0][0] +
		                dY 	* (*matrix)[0][1] +
		                dZ 	* (*matrix)[0][2] +
		                	  (*matrix)[0][3];
		dYTemp = 		dX 	* (*matrix)[1][0] +
		                dY 	* (*matrix)[1][1] +
		                dZ 	* (*matrix)[1][2] +
		                	  (*matrix)[1][3];
		dZTemp = 		dX 	* (*matrix)[2][0] +
						dY	* (*matrix)[2][1] +
						dZ	* (*matrix)[2][2] +
							  (*matrix)[2][3];
		point->dX = dXTemp;
		point->dY = dYTemp;
		point->dZ = dZTemp;
	}
}

CCartesianCloud3D* CCartesianCloud3D::createTransform(CMatrix44* matrix)
{
	double d00 = (double)(*matrix)[0][0];
	double d01 = (double)(*matrix)[0][1];
	double d02 = (double)(*matrix)[0][2];
	double d03 = (double)(*matrix)[0][3];
	double d10 = (double)(*matrix)[1][0];
	double d11 = (double)(*matrix)[1][1];
	double d12 = (double)(*matrix)[1][2];
	double d13 = (double)(*matrix)[1][3];
	double d20 = (double)(*matrix)[2][0];
	double d21 = (double)(*matrix)[2][1];
	double d22 = (double)(*matrix)[2][2];
	double d23 = (double)(*matrix)[2][3];
	
	CCartesianCloud3D* cloud = new CCartesianCloud3D();
	if(_nHasInfo!=0)
	{
		for(unsigned int i = 0; i<_vPoints.size(); i++)
		{
			StrCartesianPoint3D* point = new StrCartesianPoint3D();
			StrPointInfo* info = new StrPointInfo(_vInfo[i]);
			
			StrCartesianPoint3D* pointSrc = _vPoints[i];
			double dX = pointSrc->dX;
			double dY = pointSrc->dY;
			double dZ = pointSrc->dZ;
						
			point->dX = dX	* d00 +
			            dY 	* d01 +
			            dZ 	* d02 +
			               	  d03;
			point->dY = dX	* d10 +
			            dY 	* d11 +
			            dZ 	* d12 +
			                  d13;
			point->dZ = dX  * d20 +
					    dY	* d21 +
					    dZ	* d22 +
					   		  d23;
			
			cloud->add(point, info);
		}
	}
	else
	{		
		for(unsigned int i = 0; i<_vPoints.size(); i++)
		{
			StrCartesianPoint3D* point = new StrCartesianPoint3D();
			StrCartesianPoint3D* pointSrc = _vPoints[i];
			double dX = pointSrc->dX;
			double dY = pointSrc->dY;
			double dZ = pointSrc->dZ;
						
			point->dX = dX	* d00 +
			            dY 	* d01 +
			            dZ 	* d02 +
			               	  d03;
			point->dY = dX	* d10 +
			            dY 	* d11 +
			            dZ 	* d12 +
			                  d13;
			point->dZ = dX  * d20 +
					    dY	* d21 +
					    dZ	* d22 +
					   		  d23;
			cloud->add(point);								
		}
	}
	return cloud;
}

struct StrFairBucket
{
	double dXSum;
	double dYSum;
	double dZSum;
	double dInt;
	double dAmp;
	double dAcc;
	double dR;
	double dG;
	double dB;
	double dCnt;
	
};
void CCartesianCloud3D::getReducedPoints(CCartesianCloud3D* cloud, double dBucketEdgeLength)
{
	map<long long, StrFairBucket*> mBuckets;
	bool bIsValid = true;
	
	unsigned int i = 0;
	if(_nHasInfo)
	{
		for(i = 0; i<_vPoints.size(); i++)
		{
			StrPointInfo* info = _vInfo[i];
			bIsValid = info->bValid;

			if(bIsValid)
			{
				StrCartesianPoint3D* point = _vPoints[i];
	
				double dX = point->dX;
				double dY = point->dY;
				double dZ = point->dZ;
				double dInt = (double)info->fIntensity;
				double dAmp = (double)info->fAmplitude;
				double dAcc = info->dAccuracy;
				double dR = (double)info->afRGB[0];
				double dG = (double)info->afRGB[1];
				double dB = (double)info->afRGB[2];
				double dSx = dX / fabs(dX) * 0.5;
				double dSy = dY / fabs(dY) * 0.5;
				double dSz = dZ / fabs(dZ) * 0.5;
				
				long long llBx = (long long)(dX / dBucketEdgeLength + dSx);
				long long llBy = (long long)(dY / dBucketEdgeLength + dSy);
				long long llBz = (long long)(dZ / dBucketEdgeLength + dSz);
				
				// We assume that there are less than 10000 buckets in one dimension
				// otherwise the hash value for each bucket might not be unique!
				long long llHashValue = 100000000 * llBx + 10000 * llBy + llBz;
	
				map<long long, StrFairBucket*>::iterator itr = mBuckets.find(llHashValue);
				if( itr == mBuckets.end())
				{
					StrFairBucket* bucket = new StrFairBucket();
					bucket->dXSum = dX;
					bucket->dYSum = dY;
					bucket->dZSum = dZ;
					bucket->dInt = dInt;
					bucket->dAmp = dAmp;
					bucket->dAcc = dAcc;
					bucket->dR = dR;
					bucket->dG = dG;
					bucket->dB = dB;
					bucket->dCnt = 1.0;
					mBuckets[llHashValue] = bucket;
				}
				else
				{
					StrFairBucket* bucket = (*itr).second;
					bucket->dXSum += dX;
					bucket->dYSum += dY;
					bucket->dZSum += dZ;
					bucket->dInt += dInt;
					bucket->dAmp += dAmp;
					bucket->dAcc += dAcc;
					bucket->dR += dR;
					bucket->dG += dG;
					bucket->dB += dB;
					bucket->dCnt  += 1.0;
				}
			}
		}
						
	}
	else
	{
		for(i = 0; i<_vPoints.size(); i++)
		{
			StrCartesianPoint3D* point = _vPoints[i];

			double dX = point->dX;
			double dY = point->dY;
			double dZ = point->dZ;
			double dSx = dX / fabs(dX) * 0.5;
			double dSy = dY / fabs(dY) * 0.5;
			double dSz = dZ / fabs(dZ) * 0.5;
			
			long long llBx = (long long)(dX / dBucketEdgeLength + dSx);
			long long llBy = (long long)(dY / dBucketEdgeLength + dSy);
			long long llBz = (long long)(dZ / dBucketEdgeLength + dSz);
			
			// We assume that there are less than 10000 buckets in one dimension
			// otherwise the hash value for each bucket might not be unique!
			long long llHashValue = 100000000 * llBx + 10000 * llBy + llBz;

			map<long long, StrFairBucket*>::iterator itr = mBuckets.find(llHashValue);
			if( itr == mBuckets.end())
			{
				StrFairBucket* bucket = new StrFairBucket();
				bucket->dXSum = dX;
				bucket->dYSum = dY;
				bucket->dZSum = dZ;
				bucket->dCnt = 1.0;
				mBuckets[llHashValue] = bucket;
			}
			else
			{
				StrFairBucket* bucket = (*itr).second;
				bucket->dXSum += dX;
				bucket->dYSum += dY;
				bucket->dZSum += dZ;
				bucket->dCnt  += 1.0;
			}
		}
	}

	cloud->erase();

	if(_nHasInfo)
	{
		for( map<long long, StrFairBucket*>::iterator itr=mBuckets.begin(); itr!=mBuckets.end(); ++itr)
		{
			StrFairBucket* bucket = (*itr).second;
			StrCartesianPoint3D* point = new StrCartesianPoint3D();
			StrPointInfo* info = new StrPointInfo();
			double dCnt = bucket->dCnt;
			point->dX = bucket->dXSum / dCnt;
			point->dY = bucket->dYSum / dCnt;
			point->dZ = bucket->dZSum / dCnt;
			info->fIntensity = (float)(bucket->dInt / dCnt);
			info->fAmplitude = (float)(bucket->dAmp / dCnt);
			info->dAccuracy = bucket->dAcc / dCnt;
			info->afRGB[0] = (float)(bucket->dR / dCnt);
			info->afRGB[1] = (float)(bucket->dG / dCnt);
			info->afRGB[2] = (float)(bucket->dB / dCnt);
			info->bValid = true;
			cloud->add(point, info);
			delete bucket;
		}
	}
	else
	{
		for( map<long long, StrFairBucket*>::iterator itr=mBuckets.begin(); itr!=mBuckets.end(); ++itr)
		{
			StrFairBucket* bucket = (*itr).second;
			StrCartesianPoint3D* point = new StrCartesianPoint3D();
			double dCnt = bucket->dCnt;
			point->dX = bucket->dXSum / dCnt;
			point->dY = bucket->dYSum / dCnt;
			point->dZ = bucket->dZSum / dCnt;
			cloud->add(point);
			delete bucket;
		}
	}
	mBuckets.clear();

}

void CCartesianCloud3D::getViewVectors(CCartesianCloud3D* pCloud)
{
	pCloud->erase();
	for ( unsigned int i = 0; i < this->_vPoints.size(); ++i )
	{
		StrCartesianPoint3D* pPoint = this->_vPoints[i];
		double dLength = sqrt(pPoint->dX*pPoint->dX + pPoint->dY*pPoint->dY + pPoint->dZ*pPoint->dZ);
		
		StrCartesianPoint3D* pViewVector = new StrCartesianPoint3D();
		pViewVector->dX = pPoint->dX / dLength;
		pViewVector->dY = pPoint->dY / dLength;
		pViewVector->dZ = pPoint->dZ / dLength;
		
		if( this->hasInfo() )
		{
			StrPointInfo* pInfo = new StrPointInfo();
			(*pInfo) = (*this->getInfo(i));
			pCloud->add(pViewVector, pInfo);
		}
		else
			pCloud->add(pViewVector);
	}
}

void CCartesianCloud3D::setTranslationVector(double* dTranslation)
{
	_adTranslation[0] = dTranslation[0];
	_adTranslation[1] = dTranslation[1];
	_adTranslation[2] = dTranslation[2];	
}
	
double* CCartesianCloud3D::getTranslationVector()
{
	return _adTranslation;	
}

void CCartesianCloud3D::setRotationPoint(StrCartesianPoint3D* point)
{
	_rotPoint.dX = point->dX;
	_rotPoint.dY = point->dY;
	_rotPoint.dZ = point->dZ;
}
	
StrCartesianPoint3D* CCartesianCloud3D::getRotationPoint()
{
	return &_rotPoint;
}
	
void CCartesianCloud3D::setRotationMatrix(CMatrix44* matrix)
{
	_mRotationMatrix = (*matrix);
}
	
CMatrix44* CCartesianCloud3D::getRotationMatrix()
{
	return &_mRotationMatrix;
}

void CCartesianCloud3D::setValid(bool bValid)
{
	if(_nHasInfo)
	{
		for(unsigned int i=0; i<_vInfo.size(); i++)
		{
			StrPointInfo* info = _vInfo[i];
			info->bValid = bValid;
		}
	}	
}
	
}
