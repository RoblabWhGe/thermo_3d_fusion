#include "fair/core/math/CMatrix.h"

namespace fair
{

// Multiply the CMatrix33 by another CMatrix33
CMatrix33 &CMatrix33::operator *= (const CMatrix33 &m) 
{
  CMatrix33 t;

  for (unsigned int r = 0; r < 3; r++)
  {
	  for (unsigned int c = 0; c < 3; c++)
	  {
	    float f = 0;
	
	    f += col[r][0] * m[0][c];
	    f += col[r][1] * m[1][c];
	    f += col[r][2] * m[2][c];
	
	    t[r][c] = f;
	  }
  }

  *this = t;

  return *this;
}

// Transpose the CMatrix33
CMatrix33 &CMatrix33::transpose() 
{
  float t;

  for (unsigned int c = 0; c < 3; c++)
  {
	  for (unsigned int r = c + 1; r < 3; r++)
	  {
	    t = col[r][c];
	    col[r][c] = col[c][r];
	    col[c][r] = t;
	  } 
  }

  return *this;
}

// Invert the CMatrix33
CMatrix33 &CMatrix33::invert() 
{
  CMatrix33 a(*this);
  CMatrix33 b(IdentityMatrix33());

  unsigned int c, r;
  unsigned int cc;
  unsigned int rowMax; // Points to max abs value row in this column
  unsigned int row;
  float tmp;

  // Go through columns
  for (c=0; c<3; c++)
  {
	  // Find the row with max value in this column
	  rowMax = c;
	  for (r=c+1; r<3; r++)
	  {
	    if (fabs(a[r][c]) > fabs(a[rowMax][c]))
	    {
	    rowMax = r;
	    }
	  }
	
	  // If the max value here is 0, we can't invert.  Return identity.
	  if (a[c][rowMax] == 0.0F)
	    return(this->identity());
	
	  // Swap row "rowMax" with row "c"
	  for (cc=0; cc<3; cc++)
	  {
	    tmp = a[c][cc];
	    a[c][cc] = a[rowMax][cc];
	    a[rowMax][cc] = tmp;
	    tmp = b[c][cc];
	    b[c][cc] = b[rowMax][cc];
	    b[rowMax][cc] = tmp;
	  }
	
	  // Now everything we do is on row "c".
	  // Set the max cell to 1 by dividing the entire row by that value
	  tmp = a[c][c];
	  for (cc=0; cc<3; cc++)
	  {
	    a[c][cc] /= tmp;
	    b[c][cc] /= tmp;
	  }
	
	  // Now do the other rows, so that this column only has a 1 and 0's
	  for (row = 0; row < 3; row++)
	  {
	    if (row != c)
	    {
	    	tmp = a[row][c];
		    for (cc=0; cc<3; cc++)
		    {
		      a[row][cc] -= a[c][cc] * tmp;
		      b[row][cc] -= b[c][cc] * tmp;
		    }
	    }
	  }

  }

  *this = b;

  return *this;
}

// Return a CMatrix33 set to the identity CMatrix
CMatrix33 IdentityMatrix33() 
{
  CMatrix33 ret;

  return ret.identity();
}

// Return the transpose of the CMatrix33
CMatrix33 TransposeMatrix33(const CMatrix33 &m) 
{
  CMatrix33 ret(m);

  return ret.transpose();
}

// Return the inverted CMatrix33
CMatrix33 InvertMatrix33(const CMatrix33 &m) 
{
  CMatrix33 ret(m);

  return ret.invert();
}

// Return a 2D rotation CMatrix33
CMatrix33 RotateRadMatrix33(float rad) 
{
  CMatrix33 ret;
  float sinA, cosA;

  sinA = (float)sin(rad);
  cosA = (float)cos(rad);

  ret[0][0] = cosA; ret[0][1] = -sinA; ret[0][2] = 0.0F;
  ret[1][0] = sinA; ret[1][1] =  cosA; ret[1][2] = 0.0F;
  ret[2][0] = 0.0F; ret[2][1] =  0.0F; ret[2][2] = 1.0F;

  return ret;
}

// Return a 2D translation CMatrix33
CMatrix33 TranslateMatrix33(float x, float y) 
{
  CMatrix33 ret;

  ret.identity();
  ret[0][2] = x;
  ret[1][2] = y;

  return ret;
}

// Return a 2D/3D scale CMatrix33
CMatrix33 ScaleMatrix33(float x, float y, float z) 
{
  CMatrix33 ret;

  ret.identity();
  ret[0][0] = x;
  ret[1][1] = y;
  ret[2][2] = z;

  return ret;
}

void printMatrix33(CMatrix33* m)
{
	printf("%2.2lf %2.2lf %2.2lf\n",(*m)[0][0],(*m)[1][0],(*m)[2][0]);
	printf("%2.2lf %2.2lf %2.2lf\n",(*m)[0][1],(*m)[1][1],(*m)[2][1]);
	printf("%2.2lf %2.2lf %2.2lf\n",(*m)[0][2],(*m)[1][2],(*m)[2][2]);	
}

////////////////////////////////////////////////////////////
// CMatrix44 class
//

// Multiply the CMatrix44 by another CMatrix44
CMatrix44 &CMatrix44::operator *= (const CMatrix44 &m) 
{
  CMatrix44 t;
  for (unsigned int r = 0; r < 4; r++)
  {
	  for (unsigned int c = 0; c < 4; c++)
	  {
	    float f = 0;
	
	    f += (col[r][0] * m[0][c]);
	    f += (col[r][1] * m[1][c]);
	    f += (col[r][2] * m[2][c]);
	    f += (col[r][3] * m[3][c]);
	
	    t[r][c] = f;
	  }
  }
  *this = t;
  return *this;
}


// Transpose the CMatrix44
CMatrix44 &CMatrix44::transpose() 
{
  float t;

  for (unsigned int c = 0; c < 4; c++)
  {
	  for (unsigned int r = c + 1; r < 4; r++)
	  {
	    t = col[r][c];
	    col[r][c] = col[c][r];
	    col[c][r] = t;
	  } 
  } 

  return *this;
}

// Invert the CMatrix44
CMatrix44 &CMatrix44::invert() 
{
  CMatrix44 a(*this);
  CMatrix44 b(IdentityMatrix44());

  unsigned int r, c;
  unsigned int cc;
  unsigned int rowMax; // Points to max abs value row in this column
  unsigned int row;
  float tmp;

  // Go through columns
  for (c=0; c<4; c++)
  {

	  // Find the row with max value in this column
	  rowMax = c;
	  for (r=c+1; r<4; r++)
	  {
	    if (fabs(a[r][c]) > fabs(a[rowMax][c]))
	    {
	    rowMax = r;
	    }
	  }
	
	  // If the max value here is 0, we can't invert.  Return identity.
	  if (a[c][rowMax] == 0.0F)
	    return(identity());
	
	  // Swap row "rowMax" with row "c"
	  for (cc=0; cc<4; cc++)
	  {
	    tmp = a[c][cc];
	    a[c][cc] = a[rowMax][cc];
	    a[rowMax][cc] = tmp;
	    tmp = b[c][cc];
	    b[c][cc] = b[rowMax][cc];
	    b[rowMax][cc] = tmp;
	  }
	
	  // Now everything we do is on row "c".
	  // Set the max cell to 1 by dividing the entire row by that value
	  tmp = a[c][c];
	  for (cc=0; cc<4; cc++)
	  {
	    a[c][cc] /= tmp;
	    b[c][cc] /= tmp;
	  }
	
	  // Now do the other rows, so that this column only has a 1 and 0's
	  for (row = 0; row < 4; row++)
	  {
		if (row != c)
		{
		    tmp = a[row][c];
		    for (cc=0; cc<4; cc++)
		    {
		      a[row][cc] -= a[c][cc] * tmp;
		      b[row][cc] -= b[c][cc] * tmp;
		    }
		}
	  }
  }

  *this = b;

  return *this;
}

// Return a CMatrix44 set to the identity CMatrix
CMatrix44 IdentityMatrix44() 
{
  CMatrix44 ret;

  return ret.identity();
}

// Return the transpose of the CMatrix44
CMatrix44 TransposeMatrix44(const CMatrix44 &m) 
{
  CMatrix44 ret(m);

  return ret.transpose();
}

// Return the inverted CMatrix44
CMatrix44 InvertMatrix44(const CMatrix44 &m) 
{
  CMatrix44 ret(m);

  return ret.invert();
}

// Return a 3D axis-rotation CMatrix44
// Pass in 'x', 'y', or 'z' for the axis.
CMatrix44 RotateRadMatrix44(char axis, float rad) 
{
  CMatrix44 ret;
  float sinA, cosA;

  sinA = (float)sin(rad);
  cosA = (float)cos(rad);

  switch (axis)
  {
  case 'x':
  case 'X':
    ret[0][0] =  1.0F; ret[0][1] =  0.0F; ret[0][2] =  0.0F;
    ret[1][0] =  0.0F; ret[1][1] =  cosA; ret[1][2] = -sinA;
    ret[2][0] =  0.0F; ret[2][1] =  sinA; ret[2][2] =  cosA;
    break;

  case 'y':
  case 'Y':
    ret[0][0] =  cosA; ret[0][1] =  0.0F; ret[0][2] =  sinA;
    ret[1][0] =  0.0F; ret[1][1] =  1.0F; ret[1][2] =  0.0F;
    ret[2][0] = -sinA; ret[2][1] =  0.0F; ret[2][2] =  cosA;
    break;

  case 'z':
  case 'Z':
    ret[0][0] =  cosA; ret[0][1] = -sinA; ret[0][2] =  0.0F;
    ret[1][0] =  sinA; ret[1][1] =  cosA; ret[1][2] =  0.0F;
    ret[2][0] =  0.0F; ret[2][1] =  0.0F; ret[2][2] =  1.0F;
    break;
  }

  ret[3][0] = 0.0F; ret[3][1] = 0.0F; ret[3][2] = 0.0F;
  ret[0][3] = 0.0F;
  ret[1][3] = 0.0F;
  ret[2][3] = 0.0F;
  ret[3][3] = 1.0F;

  return ret;
}

// Return a 3D axis-rotation CMatrix44
// Pass in an arbitrary CVector3 axis.
CMatrix44 RotateRadMatrix44(const CVector3 &axis, float rad) 
{
  CMatrix44 ret;
  float sinA, cosA;
  float invCosA;
  CVector3 nrm = axis;
  float x, y, z;
  float xSq, ySq, zSq;

  nrm.normalize();
  sinA = (float)sin(rad);
  cosA = (float)cos(rad);
  invCosA = 1.0F - cosA;

  x = nrm.x;
  y = nrm.y;
  z = nrm.z;

  xSq = x * x;
  ySq = y * y;
  zSq = z * z;

  ret[0][0] = (invCosA * xSq) + (cosA);
  ret[0][1] = (invCosA * x * y) - (sinA * z );
  ret[0][2] = (invCosA * x * z) + (sinA * y );
  ret[0][3] = 0.0F;

  ret[1][0] = (invCosA * x * y) + (sinA * z);
  ret[1][1] = (invCosA * ySq) + (cosA);
  ret[1][2] = (invCosA * y * z) - (sinA * x);
  ret[1][3] = 0.0F;

  ret[2][0] = (invCosA * x * z) - (sinA * y);
  ret[2][1] = (invCosA * y * z) + (sinA * x);
  ret[2][2] = (invCosA * zSq) + (cosA);
  ret[2][3] = 0.0F;

  ret[3][0] = 0.0F;
  ret[3][1] = 0.0F;
  ret[3][2] = 0.0F;
  ret[3][3] = 1.0F;

  return ret;
}

// Return a 3D translation CMatrix44
CMatrix44 TranslateMatrix44(float x, float y, float z) 
{
  CMatrix44 ret;

  ret.identity();
  ret[0][3] = x;
  ret[1][3] = y;
  ret[2][3] = z;

  return ret;
}

// Return a 3D/4D scale CMatrix44
CMatrix44 ScaleMatrix44(float x, float y, float z, float w) 
{
  CMatrix44 ret;

  ret.identity();
  ret[0][0] = x;
  ret[1][1] = y;
  ret[2][2] = z;
  ret[3][3] = w;

  return ret;
}

// Return a "lookat" CMatrix44 given the current camera position (CVector3),
//   camera-up CVector3, and camera-target CVector3.
CMatrix44 LookAtMatrix44(const CVector3 &camPos, const CVector3 &target, 
    const CVector3 &camUp ) 
{
  CMatrix44 ret;

  CVector3 F = target - camPos;
  F.normalize();

  CVector3 S = CrossProduct(F, Normalized(camUp));
  S.normalize();

  CVector3 U = CrossProduct(S, F);
  U.normalize();

  ret[0][0] = S.x;
  ret[0][1] = S.y;
  ret[0][2] = S.z;
  ret[0][3] = 0.0;

  ret[1][0] = U.x;
  ret[1][1] = U.y;
  ret[1][2] = U.z;
  ret[1][3] = 0.0;

  ret[2][0] = -F.x;
  ret[2][1] = -F.y;
  ret[2][2] = -F.z;
  ret[2][3] = 0.0;

  ret[3][0] = 0.0F;
  ret[3][1] = 0.0F;
  ret[3][2] = 0.0F;
  ret[3][3] = 1.0F;

  ret *= TranslateMatrix44(-camPos.x, -camPos.y, -camPos.z);

  return ret;
}

// Return a frustum CMatrix44 given the left, right, bottom, top,
//   near, and far values for the frustum boundaries.
CMatrix44 FrustumMatrix44(float l, float r, 
    float b, float t, float n, float f) 
{
  CMatrix44 ret;
  float width = r-l;
  float height = t-b;
  float depth = f-n;

  ret[0][0] = (2*n) / width;
  ret[1][0] = 0.0F;
  ret[2][0] = 0.0F;
  ret[3][0] = 0.0F;

  ret[0][1] = 0.0F;
  ret[1][1] = (2*n) / height;
  ret[2][1] = 0.0F;
  ret[3][1] = 0.0F;

  ret[0][2] = (r + l) / width;
  ret[1][2] = (t + b) / height;
  ret[2][2] = -(f + n) / depth;
  ret[3][2] = -1.0F;

  ret[0][3] = 0.0F;
  ret[1][3] = 0.0F;
  ret[2][3] = -(2*f*n) / depth;
  ret[3][3] = 0.0F;

  return ret;
}

// Return a perspective CMatrix44 given the field-of-view in the Y
//   direction in degrees, the aspect ratio of Y/X, and near and
//   far plane distances.
CMatrix44 PerspectiveMatrix44(float fovY, float aspect, float n, float f) 
{
  CMatrix44 ret;
  float angle;
  float cot;

  angle = fovY / 2.0F;
  angle = DegToRad( angle );

  cot = (float) cos(angle) / (float) sin(angle);

  ret[0][0] = cot / aspect;
  ret[1][0] = 0.0F;
  ret[2][0] = 0.0F;
  ret[3][0] = 0.0F;

  ret[0][1] = 0.0F;
  ret[1][1] = cot;
  ret[2][1] = 0.0F;
  ret[3][1] = 0.0F;

  ret[0][2] = 0.0F;
  ret[1][2] = 0.0F;
  ret[2][2] = -(f + n) / (f - n);
  ret[3][2] = -1.0F;


  ret[0][3] = 0.0F;
  ret[1][3] = 0.0F;
  ret[2][3] = -(2*f*n) / (f - n);
  ret[3][3] = 0.0F;

  return ret;
}

// Return an orthographic CMatrix44 given the left, right, bottom, top,
//   near, and far values for the frustum boundaries.
CMatrix44 OrthoMatrix44(float l, float r, 
    float b, float t, float n, float f) 
{
  CMatrix44 ret;
  float width = r-l;
  float height = t-b;
  float depth = f-n;

  ret[0][0] = 2.0F / width;
  ret[1][0] = 0.0F;
  ret[2][0] = 0.0F;
  ret[3][0] = 0.0F;

  ret[0][1] = 0.0F;
  ret[1][1] = 2.0F / height;
  ret[2][1] = 0.0F;
  ret[3][1] = 0.0F;

  ret[0][2] = 0.0F;
  ret[1][2] = 0.0F;
  ret[2][2] = -(2.0F) / depth;
  ret[3][2] = 0.0F;

  ret[0][3] = -(r + l) / width;
  ret[3][1] = -(t + b) / height;
  ret[2][3] = -(f + n) / depth;
  ret[3][3] = 1.0F;

  return ret;
}

// Return an orientation CMatrix using 3 basis normalized vectors
CMatrix44    OrthoNormalMatrix44(const CVector3 &xdir, 
    const CVector3 &ydir, const CVector3 &zdir)
{
  CMatrix44 ret;

  ret[0] = (CVector4)xdir;
  ret[1] = (CVector4)ydir;
  ret[2] = (CVector4)zdir;
  ret[3][3] = 1.0F;

  return ret;
}

void printMatrix44(CMatrix44* m)
{
	printf("%2.4lf %2.4lf %2.4lf %2.4lf\n",(*m)[0][0],(*m)[0][1],(*m)[0][2],(*m)[0][3]);
	printf("%2.4lf %2.4lf %2.4lf %2.4lf\n",(*m)[1][0],(*m)[1][1],(*m)[1][2],(*m)[1][3]);
	printf("%2.4lf %2.4lf %2.4lf %2.4lf\n",(*m)[2][0],(*m)[2][1],(*m)[2][2],(*m)[2][3]);
	printf("%2.4lf %2.4lf %2.4lf %2.4lf\n",(*m)[3][0],(*m)[3][1],(*m)[3][2],(*m)[3][3]);	
}

}
