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

#ifndef CARTESIANBASE_H
#define CARTESIANBASE_H

#include "../../../fair/core/math/CMatrix.h"
#include "../../../fair/core/math/CVector.h"
#include <stdlib.h>
#include <string.h>
#include <iostream>

/**
 * @namespace fair
 */
namespace fair {

/**
 * @struct StrPointInfo
 * @brief additional informations for each point.
 * This structure is kept extra to minimize overhead for cartesian clouds without any additional informations.
 * @author Stefan May
 */
struct StrPointInfo {

		/** class to which the point belongs in semantic labeling */
		unsigned int unClass;
		/** intensity, i.e. a grey value caused by backlight illumination */
		float fIntensity;
		/** amplitude, i.e. a grey value for an active sensor like TOF cameras */
		float fAmplitude;
		/** the pixels accuracy */
		double dAccuracy;
		/** Red/Green/Blue value */
		float afRGB[3];
		/** flag indicating a valid measurement (this depends on the application's requirements) */
		bool bValid;
		/** user defined data **/
		double* pdUserData;
		/** length of user data **/
		unsigned int unUserDataLength;
		/** distance value of the point */
		double dDistance;
		/** alpha of the point raw data */
		double dAlpha;
		/** beta of the point raw data */
		double dBeta;
		fair::CVector3* normalVector;

		/** Default constructor **/
		StrPointInfo() {
			pdUserData = NULL;
			unUserDataLength = 0;
			unClass = 0;
			normalVector = NULL;
		}
		;

		/** Copy constructor **/
		StrPointInfo(StrPointInfo* info) {
			fIntensity = info->fIntensity;
			fAmplitude = info->fAmplitude;
			dAccuracy = info->dAccuracy;
			afRGB[0] = info->afRGB[0];
			afRGB[1] = info->afRGB[1];
			afRGB[2] = info->afRGB[2];
			bValid = info->bValid;
			if (info->unUserDataLength > 0) {
				pdUserData = new double[unUserDataLength];
				memcpy(pdUserData, info->pdUserData, unUserDataLength * sizeof(double));
			} else {
				pdUserData = NULL;
			}
			dDistance = info->dDistance;
			dAlpha = info->dAlpha;
			dBeta = info -> dBeta;
			normalVector = info->normalVector;
		}
		;

		void setColor(float* pfColor) {
			memcpy(afRGB, pfColor, 3 * sizeof(float));
		}
		;

		/**
		 * Operator overload. Defines the usage of the stream output operator '<<'
		 * Output looks like : Valid(1) RGB(255.0,200.0,100.5) Intensity(120.1) Amplitude(80.0) Polar(10.0,45.0,170.0)
		 * Note: Polar is structured in Polar(Distance,Alpha,dBeta)
		 */
		inline std::ostream& operator<<(std::ostream& out) {
			return out << "Valid(" << bValid << ") RGB(" << afRGB[0] << ',' << afRGB[1] << ',' << afRGB[2] << ')' << " Intensity("
					<< fIntensity << " Amplitude(" << fAmplitude << ") Polar(" << dDistance << "," << dAlpha << "," << dBeta << ")";
		}

};

/**
 * @struct StrCartesianPoint2D
 * @brief Represents a single point in 2D cartesian space
 * @author Stefan May
 **/
struct StrCartesianPoint2D {
		/** Default constructor **/
		StrCartesianPoint2D() {
		}
		;

		/** Copy constructor **/
		StrCartesianPoint2D(StrCartesianPoint2D* point) {
			dX = point->dX;
			dY = point->dY;
		}
		;

		/** copy raw data to array **/
		void getRawData(double* dBuffer) {
			dBuffer[0] = dX;
			dBuffer[1] = dY;
			//dBuffer[2] = dZ;
		}
		;

		/** x coordinate */
		double dX;
		/** y coordinate */
		double dY;
};

/**
 * @struct StrCartesianPoint3D
 * @brief Represents a single point in 3D cartesian space
 * @author Stefan May
 **/
struct StrCartesianPoint3D {
		/** Default constructor **/
		StrCartesianPoint3D() {
		}
		;

		StrCartesianPoint3D(double x, double y, double z) {
			dX = x;
			dY = y;
			dZ = z;
		}
		;

		/** Copy constructor **/
		StrCartesianPoint3D(StrCartesianPoint3D* point) {
			dX = point->dX;
			dY = point->dY;
			dZ = point->dZ;
		}
		;

		/**
		 *Operator overload. Defines the usage of the field operator '[]'.
		 *E.g.: StrCartesianPoint3D point;
		 *- point[0] returns the x-value
		 *- point[1] returns the y-value
		 *- point[2] returns the z-value
		 */
		inline double operator[](size_t const N) const {
			assert(0 <= N && 2 >= N);
			switch (N)
			{
				case 0:
					return dX;
				case 1:
					return dY;
				case 2:
				default:
					return dZ;
			}
		}

		/**
		 * Operator overload. Defines the usage of the comparative operator '=='
		 * TRUE if all components equal, else FALSE
		 */
		inline bool operator==(StrCartesianPoint3D const& A) {
			return (A.dX == dX && A.dY == dY && A.dZ == dZ);
		}

		/**
		 * Operator overload. Defines the usage of the not equal operator '!='
		 * TRUE if one components is differs, else FALSE
		 */
		inline bool operator!=(StrCartesianPoint3D const& other) const {
			return dX != other.dX || dY != other.dY || dZ != other.dZ;
		}

		/**
		 * Operator overload. Defines the usage of the stream output operator '<<'
		 * Output looks like : (1.0,2.0,3.0) or in general (dX,dY,dZ)
		 */
		inline std::ostream& operator<<(std::ostream& out) {
			return out << '(' << dX << ',' << dY << ',' << dZ << ')';
		}

		/**
		 * copy raw data to array
		 **/
		inline void getRawData(double* dBuffer) {
			dBuffer[0] = dX;
			dBuffer[1] = dY;
			dBuffer[2] = dZ;
		}

		/** x coordinate */
		double dX;
		/** y coordinate */
		double dY;
		/** z coordinate */
		double dZ;
};

/**
 * @struct TdCartesianPoint
 * @brief Represents a single point in cartesian space
 * @author Stefan May and Dirk Holz
 **/
typedef double* TdCartesianPoint;

/**
 * @struct StrCartesianIndexPair
 * @brief Representation of one pair of point indices
 * @author Stefan May
 */
struct StrCartesianIndexPair {
		/** index of first point */
		unsigned int indexFirst;
		/** index of second point */
		unsigned int indexSecond;
};

/**
 * @struct StrCartesianPair
 * @brief Represents a point pair in cartesian space
 * @author Stefan May and Dirk Holz
 **/
struct StrCartesianPair {
		/** first point's coordinates */
		TdCartesianPoint first;
		/** second point's coordinates */
		TdCartesianPoint second;
};

}

#endif /* CARTESIANBASE_H */
