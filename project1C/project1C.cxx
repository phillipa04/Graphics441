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
      unsigned char color[3];
      void Print(void) { cerr << "(" << X[0] << ", " << Y[0] << "), (" << X[1] << ", " << Y[1] << "), (" << X[2] << ", " << Y[2] << "), with color " << (int) color[0] << "/" << (int) color[1] << "/" << (int) color[2] << endl;};
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
    rdr->SetFileName("proj1c_geometry.vtk");
    cerr << "Reading" << endl;
    rdr->Update();
    if (rdr->GetOutput()->GetNumberOfCells() == 0)
    {
        cerr << "Unable to open file!!" << endl;
        exit(EXIT_FAILURE);
    }
    vtkPolyData *pd = rdr->GetOutput();
    int numTris = pd->GetNumberOfCells();
    vtkPoints *pts = pd->GetPoints();
    vtkCellArray *cells = pd->GetPolys();
    vtkFloatArray *colors = (vtkFloatArray *) pd->GetPointData()->GetArray("color_nodal");
    float *color_ptr = colors->GetPointer(0);
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
        tris[idx].color[0] = (unsigned char) color_ptr[4*ptIds[0]+0];
        tris[idx].color[1] = (unsigned char) color_ptr[4*ptIds[0]+1];
        tris[idx].color[2] = (unsigned char) color_ptr[4*ptIds[0]+2];
    }
    cerr << "Done reading" << endl;

    return tris;
}

void writeFlatBottomTriangle(Triangle triangle, unsigned char *buffer){
  double baseLeftX, baseLeftY, baseRightX, baseRightY, peakX, peakY;
  if(triangle.Y[0] == triangle.Y[1]){
    baseRightY = triangle.Y[0];
    baseLeftY  = triangle.Y[0];
    if(triangle.X[0] < triangle.X[1]){
      baseLeftX  = triangle.X[0];
      baseRightX = triangle.X[1];
    }
    else{
      baseLeftX  = triangle.X[1];
      baseRightX = triangle.X[0];
    }
    peakX = triangle.X[2];
    peakY = triangle.Y[2];
  }
  else if(triangle.Y[0] == triangle.Y[2]){
    baseRightY = triangle.Y[0];
    baseLeftY  = triangle.Y[0];
    if(triangle.X[0] < triangle.X[2]){
      baseLeftX  = triangle.X[0];
      baseRightX = triangle.X[2];
    }
    else{
      baseLeftX  = triangle.X[2];
      baseRightX = triangle.X[0];
    }
    peakX = triangle.X[1];
    peakY = triangle.Y[1];
  }
  else{
    baseRightY = triangle.Y[1];
    baseLeftY  = triangle.Y[1];
    if(triangle.X[1] < triangle.X[2]){
      baseLeftX  = triangle.X[1];
      baseRightX = triangle.X[2];
    }
    else{
      baseLeftX  = triangle.X[2];
      baseRightX = triangle.X[1];
    }
    peakX = triangle.X[0];
    peakY = triangle.Y[0];
  }

  int ymin, ymax;
  if(baseLeftY < 0){
    ymin = 0;
  }
  else{
    ymin = ceil441(baseLeftY);
  }

  if(peakY >= 1344){
    ymax = 1343;
  }
  else{
    ymax = floor441(peakY);
  }

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


  for(int i = ymin; i <= ymax; i++){
    int rightBound, leftBound, currentLeftX, currentRightX;
    if(slopeLeft != NULL){
      currentLeftX = ceil441(baseLeftX + (((double) i) - baseLeftY)/(*slopeLeft));
    }
    else{
      currentLeftX = ceil441(baseLeftX);
    }

    if(currentLeftX < 0){
      leftBound = 0;
    }
    else{
      leftBound = currentLeftX;
    }

    if(slopeRight != NULL){
      currentRightX = baseRightX + (((double) i) - baseLeftY)/(*slopeRight);
    }
    else{
      currentRightX = baseRightX;
    }

    if(currentRightX >= 1786){
      rightBound = 1785;
    }
    else{
      rightBound = currentRightX;
    }

    // printf("ymin = %i\n", ymin);
    // printf("ymax = %i\n", ymax);
    // printf("leftBound = %i\n", leftBound);
    // printf("rightBound = %i\n", rightBound);

    for(int j = leftBound; j <= rightBound; j++){
      int currentPixel = 1786*i + j;
      int currentIndex = 3*currentPixel;
      buffer[currentIndex] = triangle.color[0];
      buffer[currentIndex + 1] = triangle.color[1];
      buffer[currentIndex + 2] = triangle.color[2];
    }
  }

}

void writeFlatTopTriangle(Triangle triangle, unsigned char *buffer){
  double baseLeftX, baseLeftY, baseRightX, baseRightY, peakX, peakY;
  if(triangle.Y[0] == triangle.Y[1]){
    baseRightY = triangle.Y[0];
    baseLeftY  = triangle.Y[0];
    if(triangle.X[0] < triangle.X[1]){
      baseLeftX  = triangle.X[0];
      baseRightX = triangle.X[1];
    }
    else{
      baseLeftX  = triangle.X[1];
      baseRightX = triangle.X[0];
    }
    peakX = triangle.X[2];
    peakY = triangle.Y[2];
  }
  else if(triangle.Y[0] == triangle.Y[2]){
    baseRightY = triangle.Y[0];
    baseLeftY  = triangle.Y[0];
    if(triangle.X[0] < triangle.X[2]){
      baseLeftX  = triangle.X[0];
      baseRightX = triangle.X[2];
    }
    else{
      baseLeftX  = triangle.X[2];
      baseRightX = triangle.X[0];
    }
    peakX = triangle.X[1];
    peakY = triangle.Y[1];
  }
  else{
    baseRightY = triangle.Y[1];
    baseLeftY  = triangle.Y[1];
    if(triangle.X[1] < triangle.X[2]){
      baseLeftX  = triangle.X[1];
      baseRightX = triangle.X[2];
    }
    else{
      baseLeftX  = triangle.X[2];
      baseRightX = triangle.X[1];
    }
    peakX = triangle.X[0];
    peakY = triangle.Y[0];
  }

  int ymin, ymax;
  if(peakY < 0){
    ymin = 0;
  }
  else{
    ymin = ceil441(peakY);
  }

  if(baseLeftY >= 1344){
    ymax = 1343;
  }
  else{
    ymax = floor441(baseLeftY);
  }

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


  for(int i = ymin; i <= ymax; i++){
    int rightBound, leftBound, currentLeftX, currentRightX;
    if(slopeLeft != NULL){
      double val1 = (((double) i) - peakY)/(*slopeLeft);
      currentLeftX = ceil441(peakX + val1);
      // printf("peakX: %f\n", peakX);
      // printf("val1: %f\n", val1);
    }
    else{
      currentLeftX = ceil441(baseLeftX);
    }

    if(currentLeftX < 0){
      leftBound = 0;
    }
    else{
      leftBound = currentLeftX;
    }

    if(slopeRight != NULL){
      double val2 = (((double) i) - peakY)/(*slopeRight);
      currentRightX = floor441(peakX + val2);
      // printf("val2: %f\n", val2);
    }
    else{
      currentRightX = floor441(baseRightX);
    }

    if(currentRightX >= 1786){
      rightBound = 1785;
    }
    else{
      rightBound = currentRightX;
    }

    // printf("ymin = %i\n", ymin);
    // printf("ymax = %i\n", ymax);
    // printf("leftBound = %i\n", leftBound);
    // printf("rightBound = %i\n", rightBound);

    for(int j = leftBound; j <= rightBound; j++){
      // printf("writing to (%i, %i)\n", j, i);
      int currentPixel = 1786*i + j;
      int currentIndex = 3*currentPixel;
      buffer[currentIndex] = triangle.color[0];
      buffer[currentIndex + 1] = triangle.color[1];
      buffer[currentIndex + 2] = triangle.color[2];
    }
  }

}

int main()
{
   vtkImageData *image = NewImage(1786, 1344);
   unsigned char *buffer = 
     (unsigned char *) image->GetScalarPointer(0,0,0);
   int npixels = 1786*1344;
   for (int i = 0 ; i < npixels*3 ; i++)
       buffer[i] = 0;

   std::vector<Triangle> triangles = GetTriangles();
   
   Screen screen;
   screen.buffer = buffer;
   screen.width = 1786;
   screen.height = 1344;

   double minY, midY, maxY, maxX, minX, midX;

   int size = triangles.size();
   //for each triangle
   for(int j = 0; j<size; j++){
   //for(int j = 1600619; j<=1600619; j++){
    Triangle currentTriangle = triangles[j];
    //printf("triangle %i\n", j);
    
    if(currentTriangle.Y[0] == currentTriangle.Y[1]){
      if(currentTriangle.Y[2] > currentTriangle.Y[0]){
        writeFlatBottomTriangle(currentTriangle, buffer);
      }
      else{
        writeFlatTopTriangle(currentTriangle, buffer);
      }
    }
    else if(currentTriangle.Y[0] == currentTriangle.Y[2]){
      if(currentTriangle.Y[1] > currentTriangle.Y[0]){
        writeFlatBottomTriangle(currentTriangle, buffer);
      }
      else{
        writeFlatTopTriangle(currentTriangle, buffer);
      }
    }
    else if(currentTriangle.Y[1] == currentTriangle.Y[2]){
      if(currentTriangle.Y[0] > currentTriangle.Y[1]){
        writeFlatBottomTriangle(currentTriangle, buffer);
      }
      else{
        writeFlatTopTriangle(currentTriangle, buffer);
      }
    }
    else{
      //find mid-Y value and split triangle into flat-bottom and flat-top pieces
      minY = currentTriangle.Y[0];
      minX = currentTriangle.X[0];
      maxY = currentTriangle.Y[0];
      maxX = currentTriangle.X[0];
      midY = currentTriangle.Y[0];
      midX = currentTriangle.X[0];
      for(int i = 1; i<3; i++){
        if(currentTriangle.Y[i] > maxY){
          maxY = currentTriangle.Y[i];
          maxX = currentTriangle.X[i];
        }
        if(currentTriangle.Y[i] < minY){
          minY = currentTriangle.Y[i];
          minX = currentTriangle.X[i];
        }
      }

      for(int i = 0; i<3; i++){
        if((currentTriangle.Y[i] < maxY) && (currentTriangle.Y[i] > minY)){
          midY = currentTriangle.Y[i];
          midX = currentTriangle.X[i];
        }
      }

      double slope = (maxY-minY)/(maxX-minX);
      double intercept = slope*(-1*maxX) + maxY;
      double newX = (midY - intercept)/slope;
      
      std::vector<Triangle> rv(2);

      Triangle triangle1 = rv[0];
      Triangle triangle2 = rv[1];

      triangle1.Y[0] = maxY;
      triangle1.X[0] = maxX;
      triangle1.Y[1] = midY;
      triangle1.X[1] = midX;
      triangle1.Y[2] = midY;
      triangle1.X[2] = newX;
      triangle1.color[0] = currentTriangle.color[0];
      triangle1.color[1] = currentTriangle.color[1];
      triangle1.color[2] = currentTriangle.color[2];

      triangle2.Y[0] = minY;
      triangle2.X[0] = minX;
      triangle2.Y[1] = midY;
      triangle2.X[1] = midX;
      triangle2.Y[2] = midY;
      triangle2.X[2] = newX;
      triangle2.color[0] = currentTriangle.color[0];
      triangle2.color[1] = currentTriangle.color[1];
      triangle2.color[2] = currentTriangle.color[2];

      // cerr << "Current triangle = " << endl;
      // currentTriangle.Print();
      // cerr << "T1 = " << endl;
      // triangle1.Print();
      // cerr << "T2 = " << endl;
      // triangle2.Print();


      writeFlatBottomTriangle(triangle1, buffer);

      writeFlatTopTriangle(triangle2, buffer);
    }



    
   }
   printf("finished writing triangles to buffer\n");

   WriteImage(image, "project1Cfinished");
}


