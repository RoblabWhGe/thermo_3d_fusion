/*
*
* libFAIR, Fraunhofer Autonomous Intelligent Robotics Library
* 
* Ownership Fraunhofer Gesellschaft e. V., Munich, Germany
*
* The FAIR library [both binary and source code (if released)] is intellectual
* property owned by Fraunhofer Gesellschaft and is protected by the Creative
* Commons license by-nc-sa (http://creativecommons.org/licenses/by-nc-sa/3.0/de/).
* The ownership remains with Fraunhofer Gesellschaft.
*/


#ifndef CCARTESIANCLOUD_H
#define CCARTESIANCLOUD_H

#ifdef WIN32
#pragma warning(disable:4786)
#endif

#include <vector>
#include <map>
#include <math.h>
#include "../../../fair/core/base/common.h"
#include "../../../fair/core/base/log.h"
#include "../../../fair/core/io/stream/IOutputStream.h"
#include "../../../fair/core/io/stream/IInputStream.h"
#include "../../../fair/core/base/cartesianbase.h"
#include "../../../fair/core/math/CMatrix.h"

using namespace std;

/**
 * @namespace fair
 */
namespace fair
{

enum EnumSourceInfo{eSOURCEROWS=0, eSOURCECOLS=1};

/**
 * @class CCartesianCloud3D
 * @brief Represents a cloud of points in space
 * @author Stefan May, Dirk Holz
 **/
class CCartesianCloud3D
{
public:
	/**
	 * Standard constructor;
	 */
	CCartesianCloud3D();
	
	/**
	 * Standard destuctor
	 */
	~CCartesianCloud3D();
	
	/**
	 * static factory method for convienent instanciation
	 * @param unPoints number of points
	 * @param bHasInfo flag for point information instanciation
	 * @return the cloud
	 */
	static CCartesianCloud3D* create(unsigned int unPoints, bool bHasInfo);
	
	/**
	 * Vector of indices used for creating a new cloud out of this instance
	 * @param pvIdx reference to vector with indices
	 * @return a new instanciated 3D cloud
	 */
	CCartesianCloud3D* createSubCloud(vector<unsigned int>* pvIdx);
	
  	/**
   	* Accessor to each point via array indices.
   	* @param i point index
   	*/
	StrCartesianPoint3D* operator [] (unsigned int i);
	
	/**
	 * Accessor to each point.
	 * @param i point index
	 */
	StrCartesianPoint3D* getPoint(unsigned int i);
	
	/**
	 * Accessor to additional point informations
	 * @param i point index
	 */
	StrPointInfo* getInfo(unsigned int i);
	
	/**
	 * Copy coordinate data to buffer (ensure, that enough memory has been allocated!)
	 * @param pdBuffer pointer to data buffer in which the coordinates will be copied (x1y1z1x2y2z2...)
	 */
	void getRawCoordinates(double* pdBuffer);

	/**
	 * Copy coordinate and intensity data to buffer (ensure, that enough memory has been allocated!)
	 * @param pdBuffer pointer to data buffer in which the coordinates will be copied (x1y1z1i1x2y2z2i2...)
	 */
	void getRawCoordinatesInt(double* pdBuffer);
	
	/**
	 * Copy coordinate and user data to buffer (ensure, that enough memory has been allocated!)
	 * @param pdBuffer pointer to data buffer in which the coordinates will be copied (e.g. x1y1z1ud11ud12ud13x2y2z2ud21...)
	 */
	void getRawCoordinatesUserData(double* pdBuffer);
	 
	/**
	 * Copy cloud to local buffers
	 * @param cloud cloud to be copied
	 */
	void copy(CCartesianCloud3D* cloud);
	
	/**
	 * Copy cartesian parameters to referenced buffer
	 * @param ppdCloud two dimensional buffer instanciated externally
	 */
	void getCartesianCoordinates(double** ppdCloud);
	
	/**
	 * Copy cartesian coordinates (in right-handed coordinate frame) to referenced buffer
	 * @param ppdCloud two dimensional buffer instanciated externally
	 * @return Number of copied points
	 */
	unsigned int getCartesianCoordinatesRH(double** ppdCloud);
	
  	/**
  	 * Add a single point
  	 * @param point Point with x, y, z, intensity and color value
  	 */
  	void add(StrCartesianPoint3D* point);

	/**
	 * Shows presence of additional point info
	 * @return flag of presence
	 */
	int hasInfo();
	
	/**
	 * Shows presence of additional source information. This is e.g. a row and column width.
	 * @return flag of presence
	 */
	int hasSourceInfo();
	
  	/**
  	 * Add a single point
  	 * @param point Point with x, y, z, intensity and color value
  	 * @param info additional point information, e.g. color, ...
  	 */
  	void add(StrCartesianPoint3D* point, StrPointInfo* info);
  	
  	/**
  	 * Add additional source information
  	 * @param eSourceInfo source information identifier
  	 * @param lValue the info
  	 */
  	void addSourceInfo(EnumSourceInfo eSourceInfo, long lValue);

	/**
	 * Add point from another cloud
	 * @param cloud source cloud
	 * @param bCheckValidFlag if set to true, only valid points are added
	 */
	void add(CCartesianCloud3D* cloud, bool bCheckValidFlag=false);

	/**
	 * Add point from another cloud
	 * @param cloud source cloud
	 * @param vIdx indices of points in cloud to be added
	 */
	void add(CCartesianCloud3D* cloud, vector<unsigned int>* vIdx);
		 
  	/**
  	 * Accessor to additional source information
  	 * @param eSourceInfo source information identifier
  	 * @param plValue the info
  	 * @return success
  	 */  	
  	int getSourceInfo(EnumSourceInfo eSourceInfo, long* plValue);
  	
  	/**
  	 * Remove a specific source information
  	 * @param eSourceInfo source information identifier
  	 * @return success
  	 */
  	int removeSourceInfo(EnumSourceInfo eSourceInfo);
  	
  	/**
  	 * Deletes all invalid points
  	 */
  	void removeInvalidPoints();
  	
  	/**
  	 * Clear source info map
  	 */
  	void clearSourceInfo();
  	
  	/**
  	 * Get size of CartesianCloud
  	 * @return number of points
  	 */
  	unsigned int size();

	/**
	 * Get centroid of point cloud
	 * @return a virtual point
	 */
	StrCartesianPoint3D* getCentroid();

	/**
	 * Get centroid of valid points. If no info is provided the returned centroid is {0.0,0.0,0.0}!
	 * @return a virtual point
	 */
	StrCartesianPoint3D* getCentroidValidPoints();

	/**
	 * Transform the point cloud, i.e. translate and/or rotate it
	 * @param matrix a 4x4 translation matrix
	 */	
	void transform(fair::CMatrix44* matrix);

	/**
	 * Creates a new transformed point cloud
	 * @param matrix a 4x4 translation matrix
	 * @return transformed point cloud
	 */	
	CCartesianCloud3D* createTransform(CMatrix44* matrix);
	
	/**
	 * Get reduced points out of this cloud
	 * @param cloud cloud instance (will be erased before processing!)
	 * @param dBucketEdgeLength edge length of buckets with which of the space will be quantified
	 */
	void getReducedPoints(CCartesianCloud3D* cloud, double dBucketEdgeLength);
	
	/**
	 * Clears all points (no deletion!). This method does not clear source info!
	 */
	void clear();

	/**
	 * Erases all points. This method does not clear source info!
	 */
	void erase();
	
	/**
	 * serializes the point cloud
	 * @param stream destination output stream
	 */
	void serialize(IOutputStream* stream);
	
	/**
	 * loads a point cloud
	 * @param stream source input stream
	 */
	static CCartesianCloud3D* load(IInputStream* stream);
	
	/**
	 * Set temporary translation vector. This value does not change the coordinates.
	 * It represents only additional information that can later be made persitant by transformation.
	 * @param dTranslation double array (size=3) with x, y and z coordinate 
	 */
	void setTranslationVector(double* dTranslation);
	
	/**
	 * Get temporary translation vector.
	 * @return double array (size=3) with x, y and z coordinate 
	 */
	double* getTranslationVector();

	/**
	 * Set temporary rotation point. This value does not change the coordinates.
	 * It represents only additional information that can later be made persitant by transformation.
	 * @param point rotation point in cartesian space 
	 */
	void setRotationPoint(StrCartesianPoint3D* point);

	/**
	 * Get temporary rotation point.
	 * @return rotation point in cartesian space 
	 */	
	StrCartesianPoint3D* getRotationPoint();
	
	/**
	 * Set temporary rotation matrix. This value does not change the coordinates.
	 * It represents only additional information that can later be made persitant by transformation.
	 * @param matrix rotation matrix 
	 */
	void setRotationMatrix(CMatrix44* matrix);

	/**
	 * Get temporary rotation matrix.
	 * @return rotation matrix 
	 */	
	CMatrix44* getRotationMatrix();
	
	/**
	 * Set force flag for displaying with user defined mode in 3D-viewer
	 * @param bForce force flag (Default is false)
	 */
	void setForceDisplayUserDefined(bool bForce);
	
	/**
	 * Get force flag for displaying with user defined mode in 3D-viewer
	 * @return force flag (Default is false)
	 */
	bool getForceDisplayUserDefined();
	
	/**
	 * Set user defined point size
	 * @param fPointSize point size (0.0 = disable)
	 */
	void setPointSize(float fPointSize);
	
	/**
	 * Get user defined point size
	 */
	float getPointSize();
	
	/**
	 * Set user defined color to each point in cloud
	 */
	void setColor(float* pfColor);
	
	/**
	 * Simply print out all data
	 */
	void dump();
	
	/**
	 * Return view vector 
	 */
	void getViewVectors(CCartesianCloud3D* pCloud);
	
	/**
	 * Set valid state for each point 
	 */
	void setValid(bool bValid);
	 
private:
	/**
	 * point container 
	 */
	vector<StrCartesianPoint3D*> _vPoints;
	
	/**
	 * info container 
	 */
	vector<StrPointInfo*> _vInfo;
	
	/**
	 * Info flag signs presence of additional point information
	 */
	int _nHasInfo;
	
	/**
	 * Source info map
	 */	
	map<int, long> _mSourceInfo;
	
	/**
	 * Temporary translation
	 */
	double _adTranslation[3];
	
	/**
	 * Temporary rotation point
	 */
	StrCartesianPoint3D _rotPoint;
	
	/**
	 * Temporary rotation
	 */
	CMatrix44 _mRotationMatrix;
	
	/**
	 * Force user defined displaying flag
	 */
	 bool _bForceUserDefined;
	 
	 /**
	  * User defined point size
	  */
	 float _fPointSize;
};

}

#endif /*CCARTESIANCLOUD_H*/
