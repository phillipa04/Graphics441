#include <iostream>
#include <vtkDataSet.h>
#include <vtkImageData.h>
#include <vtkPNGWriter.h>
#include <vtkPointData.h>
#include <vtkPolyData.h>
#include <vtkPolyDataReader.h>
#include <vtkPoints.h>
#include <vtkUnsignedCharArray.h>
#include <vtkFloatArray.h>
#include <vtkCellArray.h>

using std::cerr;
using std::endl;
//Setting debug to true will print values for the triangles as a whole and you should only do this if you zero in on a single triangle
//and adjust the outer scanline FOR loop to only handle that single triangle.  The verboseDebug should be used in the same conditions
//but prints out much more detailed information
bool debug = false;
bool verboseDebug = false;

double ceil441(double f)
{
    return ceil(f-0.00001);
}

double floor441(double f)
{
    return floor(f+0.00001);
}


vtkImageData *
NewImage(int width, int height)
{
    vtkImageData *img = vtkImageData::New();
    img->SetDimensions(width, height, 1);
    img->AllocateScalars(VTK_UNSIGNED_CHAR, 3);

    return img;
}

void
WriteImage(vtkImageData *img, const char *filename)
{
   std::string full_filename = filename;
   full_filename += ".png";
   vtkPNGWriter *writer = vtkPNGWriter::New();
   writer->SetInputData(img);
   writer->SetFileName(full_filename.c_str());
   writer->Write();
   writer->Delete();
}

class Triangle
{
  public:
      double         X[3];
      double         Y[3];
      double         Z[3];
      double         color[3][3];
      void Print(void) { cerr << "(" << X[0] << ", " << Y[0] << "), (" << X[1] << ", " << Y[1] << "), (" << X[2] << ", " << Y[2] << ")" << endl;};
      // Methods for transforming the triangle in place be helpful?
};

class Screen
{
  public:
      unsigned char   *buffer;
      int width, height;

  // would some methods for accessing and setting pixels be helpful?
};

std::vector<Triangle>
GetTriangles(void)
{
    vtkPolyDataReader *rdr = vtkPolyDataReader::New();
    rdr->SetFileName("proj1d_geometry.vtk");
    cerr << "Reading" << endl;
    rdr->Update();
    cerr << "Done reading" << endl;
    if (rdr->GetOutput()->GetNumberOfCells() == 0)
    {
        cerr << "Unable to open file!!" << endl;
        exit(EXIT_FAILURE);
    }
    vtkPolyData *pd = rdr->GetOutput();
    int numTris = pd->GetNumberOfCells();
    vtkPoints *pts = pd->GetPoints();
    vtkCellArray *cells = pd->GetPolys();
    vtkFloatArray *var = (vtkFloatArray *) pd->GetPointData()->GetArray("hardyglobal");
    float *color_ptr = var->GetPointer(0);
    std::vector<Triangle> tris(numTris);
    vtkIdType npts;
    vtkIdType *ptIds;
    int idx;
    for (idx = 0, cells->InitTraversal() ; cells->GetNextCell(npts, ptIds) ; idx++)
    {
        if (npts != 3)
        {
            cerr << "Non-triangles!! ???" << endl;
            exit(EXIT_FAILURE);
        }
        tris[idx].X[0] = pts->GetPoint(ptIds[0])[0];
        tris[idx].X[1] = pts->GetPoint(ptIds[1])[0];
        tris[idx].X[2] = pts->GetPoint(ptIds[2])[0];
        tris[idx].Y[0] = pts->GetPoint(ptIds[0])[1];
        tris[idx].Y[1] = pts->GetPoint(ptIds[1])[1];
        tris[idx].Y[2] = pts->GetPoint(ptIds[2])[1];
        tris[idx].Z[0] = pts->GetPoint(ptIds[0])[2];
        tris[idx].Z[1] = pts->GetPoint(ptIds[1])[2];
        tris[idx].Z[2] = pts->GetPoint(ptIds[2])[2];
        // 1->2 interpolate between light blue, dark blue
        // 2->2.5 interpolate between dark blue, cyan
        // 2.5->3 interpolate between cyan, green
        // 3->3.5 interpolate between green, yellow
        // 3.5->4 interpolate between yellow, orange
        // 4->5 interpolate between orange, brick
        // 5->6 interpolate between brick, salmon
        double mins[7] = { 1, 2, 2.5, 3, 3.5, 4, 5 };
        double maxs[7] = { 2, 2.5, 3, 3.5, 4, 5, 6 };
        unsigned char RGB[8][3] = { { 71, 71, 219 }, 
                                    { 0, 0, 91 },
                                    { 0, 255, 255 },
                                    { 0, 128, 0 },
                                    { 255, 255, 0 },
                                    { 255, 96, 0 },
                                    { 107, 0, 0 },
                                    { 224, 76, 76 } 
                                  };
        for (int j = 0 ; j < 3 ; j++)
        {
            float val = color_ptr[ptIds[j]];
            int r;
            for (r = 0 ; r < 7 ; r++)
            {
                if (mins[r] <= val && val < maxs[r])
                    break;
            }
            if (r == 7)
            {
                cerr << "Could not interpolate color for " << val << endl;
                exit(EXIT_FAILURE);
            }
            double proportion = (val-mins[r]) / (maxs[r]-mins[r]);
            tris[idx].color[j][0] = (RGB[r][0]+proportion*(RGB[r+1][0]-RGB[r][0]))/255.0;
            tris[idx].color[j][1] = (RGB[r][1]+proportion*(RGB[r+1][1]-RGB[r][1]))/255.0;
            tris[idx].color[j][2] = (RGB[r][2]+proportion*(RGB[r+1][2]-RGB[r][2]))/255.0;
        }
    }

    return tris;
}

void writeFlatBottomTriangle(Triangle triangle, unsigned char *buffer, int width, int height, double* zVals){
  if(debug){
    printf("flat bottom\n");
  }
  double baseLeftX, baseLeftY, baseLeftZ, baseRightX, baseRightY, baseRightZ, peakX, peakY, peakZ;
  double baseLeftRed, baseLeftGreen, baseLeftBlue, baseRightRed, baseRightGreen, baseRightBlue, peakRed, peakGreen, peakBlue;
  double currentLeftRed, currentLeftGreen, currentLeftBlue, currentRightRed, currentRightGreen, currentRightBlue;
  double currentRed, currentGreen, currentBlue;


  //determine which vertices are in the base and which is in the peak
  if(triangle.Y[0] == triangle.Y[1]){
    baseRightY = triangle.Y[0];
    baseLeftY  = triangle.Y[0];
    if(triangle.X[0] < triangle.X[1]){
      baseLeftX     = triangle.X[0];
      baseLeftZ     = triangle.Z[0];
      baseLeftRed   = triangle.color[0][0];
      baseLeftGreen = triangle.color[0][1];
      baseLeftBlue  = triangle.color[0][2];

      baseRightX     = triangle.X[1];
      baseRightZ     = triangle.Z[1];
      baseRightRed   = triangle.color[1][0];
      baseRightGreen = triangle.color[1][1];
      baseRightBlue  = triangle.color[1][2];
    }
    else{
      baseLeftX  = triangle.X[1];
      baseLeftZ  = triangle.Z[1];
      baseLeftRed   = triangle.color[1][0];
      baseLeftGreen = triangle.color[1][1];
      baseLeftBlue  = triangle.color[1][2];

      baseRightX = triangle.X[0];
      baseRightZ = triangle.Z[0];
      baseRightRed   = triangle.color[0][0];
      baseRightGreen = triangle.color[0][1];
      baseRightBlue  = triangle.color[0][2];
    }
    peakX = triangle.X[2];
    peakY = triangle.Y[2];
    peakZ = triangle.Z[2];
    peakRed   = triangle.color[2][0];
    peakGreen = triangle.color[2][1];
    peakBlue  = triangle.color[2][2];
  }
  else if(triangle.Y[0] == triangle.Y[2]){
    baseRightY = triangle.Y[0];
    baseLeftY  = triangle.Y[0];
    if(triangle.X[0] < triangle.X[2]){
      baseLeftX  = triangle.X[0];
      baseLeftZ  = triangle.Z[0];
      baseLeftRed   = triangle.color[0][0];
      baseLeftGreen = triangle.color[0][1];
      baseLeftBlue  = triangle.color[0][2];

      baseRightX = triangle.X[2];
      baseRightZ = triangle.Z[2];
      baseRightRed   = triangle.color[2][0];
      baseRightGreen = triangle.color[2][1];
      baseRightBlue  = triangle.color[2][2];
    }
    else{
      baseLeftX  = triangle.X[2];
      baseLeftZ  = triangle.Z[2];
      baseLeftRed   = triangle.color[2][0];
      baseLeftGreen = triangle.color[2][1];
      baseLeftBlue  = triangle.color[2][2];

      baseRightX = triangle.X[0];
      baseRightZ = triangle.Z[0];
      baseRightRed   = triangle.color[0][0];
      baseRightGreen = triangle.color[0][1];
      baseRightBlue  = triangle.color[0][2];
    }
    peakX = triangle.X[1];
    peakY = triangle.Y[1];
    peakZ = triangle.Z[1];
    peakRed   = triangle.color[1][0];
    peakGreen = triangle.color[1][1];
    peakBlue  = triangle.color[1][2];
  }
  else{
    baseRightY = triangle.Y[1];
    baseLeftY  = triangle.Y[1];
    if(triangle.X[1] < triangle.X[2]){
      baseLeftX  = triangle.X[1];
      baseLeftZ  = triangle.Z[1];
      baseLeftRed   = triangle.color[1][0];
      baseLeftGreen = triangle.color[1][1];
      baseLeftBlue  = triangle.color[1][2];

      baseRightX = triangle.X[2];
      baseRightZ = triangle.Z[2];
      baseRightRed   = triangle.color[2][0];
      baseRightGreen = triangle.color[2][1];
      baseRightBlue  = triangle.color[2][2];
    }
    else{
      baseLeftX  = triangle.X[2];
      baseLeftZ  = triangle.Z[2];
      baseLeftRed   = triangle.color[2][0];
      baseLeftGreen = triangle.color[2][1];
      baseLeftBlue  = triangle.color[2][2];

      baseRightX = triangle.X[1];
      baseRightZ = triangle.Z[1];
      baseRightRed   = triangle.color[1][0];
      baseRightGreen = triangle.color[1][1];
      baseRightBlue  = triangle.color[1][2];
    }
    peakX = triangle.X[0];
    peakY = triangle.Y[0];
    peakZ = triangle.Z[0];
    peakRed   = triangle.color[0][0];
    peakGreen = triangle.color[0][1];
    peakBlue  = triangle.color[0][2];
  }

  //determine range for y values
  int ymin, ymax;
  if(baseLeftY < 0){
    ymin = 0;
  }
  else{
    ymin = ceil441(baseLeftY);
  }

  if(peakY >= height){
    ymax = height - 1;
  }
  else{
    ymax = floor441(peakY);
  }


  //determine the slope of the non-base edges, NULL represents infinite slope (vertical line)
  double* slopeLeft;
  double* slopeRight;

  if(peakX == baseLeftX){
    slopeLeft = NULL;
  }
  else{
    double val1 = (peakY - baseLeftY)/(peakX - baseLeftX);
    slopeLeft = &val1;
  }
  if(peakX == baseRightX){
    slopeRight = NULL;
  }
  else{
    double val2 = (baseRightY - peakY)/(baseRightX - peakX);
    slopeRight = &val2;
  }

  if(debug){
    printf("peak RGB: %f/%f/%f, at (%f, %f)\n", peakRed, peakGreen, peakBlue, peakX, peakY);
    printf("left base RGB: %f/%f/%f, at (%f, %f)\n", baseLeftRed, baseLeftGreen, baseLeftBlue, baseLeftX, baseLeftY);
    printf("right base RGB: %f/%f/%f, at (%f, %f)\n\n", baseRightRed, baseRightGreen, baseRightBlue, baseRightX, baseRightY);

    printf("ymin: %i\n", ymin);
    printf("ymax: %i\n\n", ymax);    
  }

  //outer loop of scanline
  for(int i = ymin; i <= ymax; i++){
    int rightBound, leftBound, leftZ, rightZ;
    double currentLeftX, currentRightX, currentLeftZ, currentRightZ, currentZ;

    //if slope = NULL then we have a right triangle and the right bound doesn't change
    if(slopeLeft != NULL){
      currentLeftX = baseLeftX + (((double) i) - baseLeftY)/(*slopeLeft);
    }
    else{
      currentLeftX = baseLeftX;
    }

    //ensure that writes do not go outside the image bounds
    if(currentLeftX < 0){
      leftBound = 0;
    }
    else{
      leftBound = ceil441(currentLeftX);
    }

    //if slope = NULL then we have a right triangle and the right bound doesn't change
    if(slopeRight != NULL){
      currentRightX = baseRightX + (((double) i) - baseLeftY)/(*slopeRight);
    }
    else{
      currentRightX = baseRightX;
    }

    //check if out of bounds of image and floor to integer
    if(currentRightX >= width){
      rightBound = width - 1;
    }
    else{
      rightBound = floor441(currentRightX);
    }

    //determine Z values and RGB values for left boundary using base-left point and peak and interpolating between them
    if(peakZ == baseLeftZ){
      currentLeftZ = baseLeftZ;
    }
    else{
      currentLeftZ = baseLeftZ + ((peakZ - baseLeftZ)/(peakY - baseLeftY))*((double) i - baseRightY);
    }
    if(peakRed == baseLeftRed){
      currentLeftRed = baseLeftRed;
    }
    else{
      currentLeftRed = baseLeftRed + ((peakRed - baseLeftRed)/(peakY - baseLeftY))*((double) i - baseRightY);
    }
    if(peakGreen == baseLeftGreen){
      currentLeftGreen = baseLeftGreen;
    }
    else{
      currentLeftGreen = baseLeftGreen + ((peakGreen - baseLeftGreen)/(peakY - baseLeftY))*((double) i - baseRightY);
    }
    if(peakBlue == baseLeftBlue){
      currentLeftBlue = baseLeftBlue;
    }
    else{
      currentLeftBlue = baseLeftBlue + ((peakBlue - baseLeftBlue)/(peakY - baseLeftY))*((double) i - baseRightY);
    }

    //same interpolation as above but for right boundary
    if(peakZ == baseRightZ){
      currentRightZ = baseRightZ;
    }
    else{
      currentRightZ = baseRightZ + ((peakZ - baseRightZ)/(peakY - baseRightY))*((double) i - baseRightY);
    }
    if(peakRed == baseRightRed){
      currentRightRed   = baseRightRed;
    }
    else{
      currentRightRed = baseRightRed + ((peakRed - baseRightRed)/(peakY - baseRightY))*((double) i - baseRightY);
    }
    if(peakGreen == baseRightGreen){
      currentRightGreen = baseRightGreen;
    }
    else{
      currentRightGreen = baseRightGreen + ((peakGreen - baseRightGreen)/(peakY - baseRightY))*((double) i - baseRightY);
    }
    if(peakBlue == baseRightBlue){
      currentRightBlue  = baseRightBlue;
    }
    else{
      currentRightBlue = baseRightBlue + ((peakBlue - baseRightBlue)/(peakY - baseRightY))*((double) i - baseRightY);
    }

    //verbose debugging prints out the scanline row (current Y value) and the left and right boundaries for that scanline
    if(verboseDebug){
      printf("baseLeftX: %f\n", baseLeftX);
      printf("currentLeftX: %f\n", currentLeftX);
      printf("left bound: %i\n\n", leftBound);

      printf("baseRightX: %f\n", baseRightX);
      printf("currentRightX: %f\n", currentRightX);
      printf("right bound: %i\n\n", rightBound);

      printf("scanline %i has leftRGB = %f, %f, %f and rightRGB = %f, %f, %f\n\n", i, currentLeftRed, currentLeftGreen, currentLeftBlue, currentRightRed, currentRightGreen, currentRightBlue);
    }

    //inner scanline loop
    for(int j = leftBound; j <= rightBound; j++){
      //determine current z and rgb values through interpolation between left and right boundaries (determined above)
      currentZ     = ((currentRightZ - currentLeftZ)/(currentRightX - currentLeftX))*((double) j - currentLeftX) + currentLeftZ;
      currentRed   = ((currentRightRed - currentLeftRed)/(currentRightX - currentLeftX))*((double) j - currentLeftX) + currentLeftRed;
      currentGreen = ((currentRightGreen - currentLeftGreen)/(currentRightX - currentLeftX))*((double) j - currentLeftX) + currentLeftGreen;
      currentBlue  = ((currentRightBlue - currentLeftBlue)/(currentRightX - currentLeftX))*((double) j - currentLeftX) + currentLeftBlue;

      //convert 2D representation of image to 1D array
      int currentPixel = width*i + j;

      //3 channels per pixel
      int currentIndex = 3*currentPixel;

      //more verbose debugging information
      if(verboseDebug){
        
      }

      if(verboseDebug){
        printf("currentZ: %f, old Z: %f\n", currentZ, zVals[currentPixel]);
      }

      //make sure current Z value is greater than what is already written and within z-bounds otherwise don't write
      if((currentZ >= zVals[currentPixel]) && (currentZ <= 0.0) && (currentZ >= -1.0)){
        int red, green, blue;
        red = ceil441(255*currentRed);
        green = ceil441(255*currentGreen);
        blue = ceil441(255*currentBlue);

        buffer[currentIndex] = red;
        buffer[currentIndex + 1] = green;
        buffer[currentIndex + 2] = blue;

        //update z-value because a new Z was written to this pixel
        zVals[currentPixel] = currentZ;
      }
    }
  }
}

void writeFlatTopTriangle(Triangle triangle, unsigned char *buffer, int width, int height, double* zVals){
  // printf("flat top\n");
  double baseLeftX, baseLeftY, baseLeftZ, baseRightX, baseRightY, baseRightZ, peakX, peakY, peakZ;
  double baseLeftRed, baseLeftGreen, baseLeftBlue, baseRightRed, baseRightGreen, baseRightBlue, peakRed, peakGreen, peakBlue;
  double currentLeftRed, currentLeftGreen, currentLeftBlue, currentRightRed, currentRightGreen, currentRightBlue;
  double currentRed, currentGreen, currentBlue;

  //determine values for base-left, base-right, and peak
  if(triangle.Y[0] == triangle.Y[1]){
    baseRightY = triangle.Y[0];
    baseLeftY  = triangle.Y[0];
    if(triangle.X[0] < triangle.X[1]){
      baseLeftX     = triangle.X[0];
      baseLeftZ     = triangle.Z[0];
      baseLeftRed   = triangle.color[0][0];
      baseLeftGreen = triangle.color[0][1];
      baseLeftBlue  = triangle.color[0][2];

      baseRightX     = triangle.X[1];
      baseRightZ     = triangle.Z[1];
      baseRightRed   = triangle.color[1][0];
      baseRightGreen = triangle.color[1][1];
      baseRightBlue  = triangle.color[1][2];
    }
    else{
      baseLeftX     = triangle.X[1];
      baseLeftZ     = triangle.Z[1];
      baseLeftRed   = triangle.color[1][0];
      baseLeftGreen = triangle.color[1][1];
      baseLeftBlue  = triangle.color[1][2];

      baseRightX     = triangle.X[0];
      baseRightZ     = triangle.Z[0];
      baseRightRed   = triangle.color[0][0];
      baseRightGreen = triangle.color[0][1];
      baseRightBlue  = triangle.color[0][2];
    }
    peakX     = triangle.X[2];
    peakY     = triangle.Y[2];
    peakZ     = triangle.Z[2];
    peakRed   = triangle.color[2][0];
    peakGreen = triangle.color[2][1];
    peakBlue  = triangle.color[2][2];
  }
  else if(triangle.Y[0] == triangle.Y[2]){
    baseRightY = triangle.Y[0];
    baseLeftY  = triangle.Y[0];
    if(triangle.X[0] < triangle.X[2]){
      baseLeftX     = triangle.X[0];
      baseLeftZ     = triangle.Z[0];
      baseLeftRed   = triangle.color[0][0];
      baseLeftGreen = triangle.color[0][1];
      baseLeftBlue  = triangle.color[0][2];

      baseRightX     = triangle.X[2];
      baseRightZ     = triangle.Z[2];
      baseRightRed   = triangle.color[2][0];
      baseRightGreen = triangle.color[2][1];
      baseRightBlue  = triangle.color[2][2];
    }
    else{
      baseLeftX     = triangle.X[2];
      baseLeftZ     = triangle.Z[2];
      baseLeftRed   = triangle.color[2][0];
      baseLeftGreen = triangle.color[2][1];
      baseLeftBlue  = triangle.color[2][2];

      baseRightX     = triangle.X[0];
      baseRightZ     = triangle.Z[0];
      baseRightRed   = triangle.color[0][0];
      baseRightGreen = triangle.color[0][1];
      baseRightBlue  = triangle.color[0][2];
    }
    peakX = triangle.X[1];
    peakY = triangle.Y[1];
    peakZ = triangle.Z[1];
    peakRed   = triangle.color[1][0];
    peakGreen = triangle.color[1][1];
    peakBlue  = triangle.color[1][2];
  }
  else{
    baseRightY = triangle.Y[1];
    baseLeftY  = triangle.Y[1];
    if(triangle.X[1] < triangle.X[2]){
      baseLeftX     = triangle.X[1];
      baseLeftZ     = triangle.Z[1];
      baseLeftRed   = triangle.color[1][0];
      baseLeftGreen = triangle.color[1][1];
      baseLeftBlue  = triangle.color[1][2];

      baseRightX     = triangle.X[2];
      baseRightZ     = triangle.Z[2];
      baseRightRed   = triangle.color[2][0];
      baseRightGreen = triangle.color[2][1];
      baseRightBlue  = triangle.color[2][2];
    }
    else{
      baseLeftX     = triangle.X[2];
      baseLeftZ     = triangle.Z[2];
      baseLeftRed   = triangle.color[2][0];
      baseLeftGreen = triangle.color[2][1];
      baseLeftBlue  = triangle.color[2][2];

      baseRightX     = triangle.X[1];
      baseRightZ     = triangle.Z[1];
      baseRightRed   = triangle.color[1][0];
      baseRightGreen = triangle.color[1][1];
      baseRightBlue  = triangle.color[1][2];
    }
    peakX     = triangle.X[0];
    peakY     = triangle.Y[0];
    peakZ     = triangle.Z[0];
    peakRed   = triangle.color[0][0];
    peakGreen = triangle.color[0][1];
    peakBlue  = triangle.color[0][2];
  }

  //determine range for scanlines making sure the values don't go outside the bounds of the image
  int ymin, ymax;
  if(peakY < 0){
    ymin = 0;
  }
  else{
    ymin = ceil441(peakY);
  }

  if(baseLeftY >= height){
    ymax = height - 1;
  }
  else{
    ymax = floor441(baseLeftY);
  }

  //determine slope of left and right edges, NULL represents vertical line (infinite slope)
  double* slopeLeft;
  double* slopeRight;
  if(peakX == baseLeftX){
    slopeLeft = NULL;
  }
  else{
    double val1 = (peakY - baseLeftY)/(peakX - baseLeftX);
    slopeLeft = &val1;
  }
  if(peakX == baseRightX){
    slopeRight = NULL;
  }
  else{
    double val2 = (baseRightY - peakY)/(baseRightX - peakX);
    slopeRight = &val2;
  }

  //debugging information
  if(debug){
    printf("peak RGB: %f/%f/%f, at (%f, %f)\n", peakRed, peakGreen, peakBlue, peakX, peakY);
    printf("left base RGB: %f/%f/%f, at (%f, %f)\n", baseLeftRed, baseLeftGreen, baseLeftBlue, baseLeftX, baseLeftY);
    printf("right base RGB: %f/%f/%f, at (%f, %f)\n\n", baseRightRed, baseRightGreen, baseRightBlue, baseRightX, baseRightY);

    printf("ymin: %i\n", ymin);
    printf("ymax: %i\n\n", ymax);    
  }

  //outer scanline loop
  for(int i = ymin; i <= ymax; i++){
    int rightBound, leftBound; 
    double currentLeftX, currentRightX, currentLeftZ, currentRightZ, currentZ;


    //determine left bound for this scanline iteration, if slope = NULL (vertical line) then it doesn't vary from base value
    if(slopeLeft != NULL){
      double val1 = (((double) i) - peakY)/(*slopeLeft);
      currentLeftX = peakX + val1;
    }
    else{
      currentLeftX = baseLeftX;
    }

    //make sure no writes outside of the bounds of the image
    if(currentLeftX < 0){
      leftBound = 0;
    }
    else{
      leftBound = ceil441(currentLeftX);
    }

    //same as above but for right bound
    if(slopeRight != NULL){
      double val2 = (((double) i) - peakY)/(*slopeRight);
      currentRightX = peakX + val2;
    }
    else{
      currentRightX = baseRightX;
    }

    //make sure doesn't write off the other side either
    if(currentRightX >= width){
      rightBound = width - 1;
    }
    else{
      rightBound = floor441(currentRightX);
    }

    //determine Z values and RGB values for left boundary using base-left point and peak and interpolating between them
    if(peakZ == baseLeftZ){
      currentLeftZ = baseLeftZ;
    }
    else{
      currentLeftZ = baseLeftZ + ((peakZ - baseLeftZ)/(peakY - baseLeftY))*((double) i - baseRightY);
    }
    if(peakRed == baseLeftRed){
      currentLeftRed = baseLeftRed;
    }
    else{
      currentLeftRed = baseLeftRed + ((peakRed - baseLeftRed)/(peakY - baseLeftY))*((double) i - baseRightY);
    }
    if(peakGreen == baseLeftGreen){
      currentLeftGreen = baseLeftGreen;
    }
    else{
      currentLeftGreen = baseLeftGreen + ((peakGreen - baseLeftGreen)/(peakY - baseLeftY))*((double) i - baseRightY);
    }
    if(peakBlue == baseLeftBlue){
      currentLeftBlue = baseLeftBlue;
    }
    else{
      currentLeftBlue = baseLeftBlue + ((peakBlue - baseLeftBlue)/(peakY - baseLeftY))*((double) i - baseRightY);
    }

    //same interpolation as above but for right boundary
    if(peakZ == baseRightZ){
      currentRightZ = baseRightZ;
    }
    else{
      currentRightZ = baseRightZ + ((peakZ - baseRightZ)/(peakY - baseRightY))*((double) i - baseRightY);
    }
    if(peakRed == baseRightRed){
      currentRightRed   = baseRightRed;
    }
    else{
      currentRightRed = baseRightRed + ((peakRed - baseRightRed)/(peakY - baseRightY))*((double) i - baseRightY);
    }
    if(peakGreen == baseRightGreen){
      currentRightGreen = baseRightGreen;
    }
    else{
      currentRightGreen = baseRightGreen + ((peakGreen - baseRightGreen)/(peakY - baseRightY))*((double) i - baseRightY);
    }
    if(peakBlue == baseRightBlue){
      currentRightBlue  = baseRightBlue;
    }
    else{
      currentRightBlue = baseRightBlue + ((peakBlue - baseRightBlue)/(peakY - baseRightY))*((double) i - baseRightY);
    }
    
    //debugging information
    if(verboseDebug){
      printf("baseLeftX: %f\n", baseLeftX);
      printf("currentLeftX: %f\n", currentLeftX);
      printf("left bound: %i\n\n", leftBound);

      printf("baseRightX: %f\n", baseRightX);
      printf("currentRightX: %f\n", currentRightX);
      printf("right bound: %i\n\n", rightBound);

      printf("scanline %i has leftRGB = %f, %f, %f and rightRGB = %f, %f, %f\n\n", i, currentLeftRed, currentLeftGreen, currentLeftBlue, currentRightRed, currentRightGreen, currentRightBlue);
    }
  
    //innner scanline loop
    for(int j = leftBound; j <= rightBound; j++){
      currentZ     = ((currentRightZ - currentLeftZ)/(currentRightX - currentLeftX))*(j - currentLeftX) + currentLeftZ;
      currentRed   = ((currentRightRed - currentLeftRed)/(currentRightX - currentLeftX))*(j - currentLeftX) + currentLeftRed;
      currentGreen = ((currentRightGreen - currentLeftGreen)/(currentRightX - currentLeftX))*(j - currentLeftX) + currentLeftGreen;
      currentBlue  = ((currentRightBlue - currentLeftBlue)/(currentRightX - currentLeftX))*(j - currentLeftX) + currentLeftBlue;

      //convert 2D representation of image to 1D array
      int currentPixel = width*i + j;

      //3 channels per pixel
      int currentIndex = 3*currentPixel;

      if(verboseDebug){
        printf("currentZ: %f, old Z: %f\n", currentZ, zVals[currentPixel]);
      }

      //make sure pixel's z value is higher than what is already written there and isn't outside of the z-bounds, otherwise don't write
      if((currentZ >= zVals[currentPixel]) && (currentZ <= 0.0) && (currentZ >= -1.0)){
        int red, green, blue;
        red = ceil441(255*currentRed);
        green = ceil441(255*currentGreen);
        blue = ceil441(255*currentBlue);

        buffer[currentIndex] = red;
        buffer[currentIndex + 1] = green;
        buffer[currentIndex + 2] = blue;

        zVals[currentPixel] = currentZ;
      }
    }
  }

}

int main()
{
  //width and height of the image
   int width = 1000;
   int height = 1000;
   vtkImageData *image = NewImage(width, height);
   unsigned char *buffer = 
     (unsigned char *) image->GetScalarPointer(0,0,0);
   int npixels = width*height;
   for (int i = 0 ; i < npixels*3 ; i++)
       buffer[i] = 0;

   std::vector<Triangle> triangles = GetTriangles();
   double zVals[npixels];

   std::fill_n(zVals, npixels, -1.0);

   
   
   Screen screen;
   screen.buffer = buffer;
   screen.width = width;
   screen.height = height;

   double minY, midY, maxY, maxX, minX, midX, maxZ, minZ, midZ, minRed, midRed, maxRed, minGreen, midGreen, maxGreen, minBlue, midBlue, maxBlue;

   int size = triangles.size();
   //for each triangle
  for(int j = 0; j<size; j++){
  //for(int j = 28273; j<=28273; j++){
    // triangleFlag = false;
    Triangle currentTriangle = triangles[j];
    //printf("triangle %i\n", j);
    
    //if any of the sides have the same y-value, determine if it is a flat-top or flat-bottom triangle and call appropriate method
    if(currentTriangle.Y[0] == currentTriangle.Y[1]){
      if(currentTriangle.Y[2] > currentTriangle.Y[0]){
        writeFlatBottomTriangle(currentTriangle, buffer, width, height, zVals);
      }
      else{
        writeFlatTopTriangle(currentTriangle, buffer, width, height, zVals);
      }
    }
    else if(currentTriangle.Y[0] == currentTriangle.Y[2]){
      if(currentTriangle.Y[1] > currentTriangle.Y[0]){
        writeFlatBottomTriangle(currentTriangle, buffer, width, height, zVals);
      }
      else{
        writeFlatTopTriangle(currentTriangle, buffer, width, height, zVals);
      }
    }
    else if(currentTriangle.Y[1] == currentTriangle.Y[2]){
      if(currentTriangle.Y[0] > currentTriangle.Y[1]){
        writeFlatBottomTriangle(currentTriangle, buffer, width, height, zVals);
      }
      else{
        writeFlatTopTriangle(currentTriangle, buffer, width, height, zVals);
      }
    }

    //otherwise split the triangle at the mid-Y value into a flat-top and flat-bottom triangle
    else{
      //initialize values
      minY = currentTriangle.Y[0];
      maxY = currentTriangle.Y[0];
      midY = currentTriangle.Y[0];
      minX = currentTriangle.X[0];
      maxX = currentTriangle.X[0];
      midX = currentTriangle.X[0];
      minZ = currentTriangle.Z[0];
      maxZ = currentTriangle.Z[0];
      midZ = currentTriangle.Z[0];
      maxRed = currentTriangle.color[0][0];
      maxGreen = currentTriangle.color[0][1];
      maxBlue = currentTriangle.color[0][2];
      minRed = currentTriangle.color[0][0];
      minGreen = currentTriangle.color[0][1];
      minBlue = currentTriangle.color[0][2];
      midRed = currentTriangle.color[0][0];
      midGreen = currentTriangle.color[0][1];
      midBlue = currentTriangle.color[0][2];

      //find min mid and max y-values and their corresponding x, z, and rgb values
      for(int i = 1; i<3; i++){
        if(currentTriangle.Y[i] > maxY){
          maxY = currentTriangle.Y[i];
          maxX = currentTriangle.X[i];
          maxZ = currentTriangle.Z[i];
          maxRed = currentTriangle.color[i][0];
          maxGreen = currentTriangle.color[i][1];
          maxBlue = currentTriangle.color[i][2];
        }
        if(currentTriangle.Y[i] < minY){
          minY = currentTriangle.Y[i];
          minX = currentTriangle.X[i];
          minZ = currentTriangle.Z[i];
          minRed = currentTriangle.color[i][0];
          minGreen = currentTriangle.color[i][1];
          minBlue = currentTriangle.color[i][2];
        }
      }

      for(int i = 0; i<3; i++){
        if((currentTriangle.Y[i] < maxY) && (currentTriangle.Y[i] > minY)){
          midY = currentTriangle.Y[i];
          midX = currentTriangle.X[i];
          midZ = currentTriangle.Z[i];
          midRed = currentTriangle.color[i][0];
          midGreen = currentTriangle.color[i][1];
          midBlue = currentTriangle.color[i][2];
        }
      }

      //debugging information
      if(debug){
        printf("V[0] = (%f, %f, %f)\n", currentTriangle.X[0], currentTriangle.Y[0], currentTriangle.Z[0]);
        printf("V[1] = (%f, %f, %f)\n", currentTriangle.X[1], currentTriangle.Y[1], currentTriangle.Z[1]);
        printf("V[2] = (%f, %f, %f)\n", currentTriangle.X[2], currentTriangle.Y[2], currentTriangle.Z[2]);
      }
      

      double newX, newZ, newRed, newGreen, newBlue;

      //determine the x, z, and rgb values for the new point, generated by splitting the triangle in two, through interpolation
      newX = minX + ((maxX - minX)/(maxY - minY))*(midY - minY);
      newZ = minZ + ((maxZ - minZ)/(maxY - minY))*(midY - minY);
      newRed = minRed + ((maxRed - minRed)/(maxY - minY))*(midY - minY);
      newGreen = minGreen + ((maxGreen - minGreen)/(maxY - minY))*(midY - minY);
      newBlue = minBlue + ((maxBlue - minBlue)/(maxY - minY))*(midY - minY);

    
      //create 2 new triangle objects to hold the 2 new triangles generated from the split
      std::vector<Triangle> rv(2);

      Triangle triangle1 = rv[0];
      Triangle triangle2 = rv[1];

      triangle1.Y[0] = maxY;
      triangle1.X[0] = maxX;
      triangle1.Z[0] = maxZ;

      triangle1.Y[1] = midY;
      triangle1.X[1] = midX;
      triangle1.Z[1] = midZ;

      triangle1.Y[2] = midY;
      triangle1.X[2] = newX;
      triangle1.Z[2] = newZ;

      triangle1.color[0][0] = maxRed;
      triangle1.color[0][1] = maxGreen;
      triangle1.color[0][2] = maxBlue;
      triangle1.color[1][0] = midRed;
      triangle1.color[1][1] = midGreen;
      triangle1.color[1][2] = midBlue;
      triangle1.color[2][0] = newRed;
      triangle1.color[2][1] = newGreen;
      triangle1.color[2][2] = newBlue;

      triangle2.Y[0] = minY;
      triangle2.X[0] = minX;
      triangle2.Z[0] = minZ;

      triangle2.Y[1] = midY;
      triangle2.X[1] = midX;
      triangle2.Z[1] = midZ;

      triangle2.Y[2] = midY;
      triangle2.X[2] = newX;
      triangle2.Z[2] = newZ;

      triangle2.color[0][0] = minRed;
      triangle2.color[0][1] = minGreen;
      triangle2.color[0][2] = minBlue;
      triangle2.color[1][0] = midRed;
      triangle2.color[1][1] = midGreen;
      triangle2.color[1][2] = midBlue;
      triangle2.color[2][0] = newRed;
      triangle2.color[2][1] = newGreen;
      triangle2.color[2][2] = newBlue;

      //debugging information
      if(debug){
        cerr << "Current triangle = " << endl;
        currentTriangle.Print();
        cerr << "T1 = " << endl;
        triangle1.Print();
        cerr << "T2 = " << endl;
        triangle2.Print();
      }
      

      //triangle1 contains the maxY so it is guaranteed to be flat-bottom (peak is higher than base)
      writeFlatBottomTriangle(triangle1, buffer, width, height, zVals);

      //triangle2 contains the minY so it is guaranteed to be flat-top (peak is lower than base)
      writeFlatTopTriangle(triangle2, buffer, width, height, zVals);
    }

   }
   printf("finished writing triangles to buffer\n");

   WriteImage(image, "project1Dfinished");
}


