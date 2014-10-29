#include <iostream>
#include <vtkDataSet.h>
#include <vtkImageData.h>
#include <vtkPNGWriter.h>
#include <vtkPointData.h>

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

  // would some methods for transforming the triangle in place be helpful?
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
   std::vector<Triangle> rv(100);

   unsigned char colors[6][3] = { {255,128,0}, {255, 0, 127}, {0,204,204}, 
                                  {76,153,0}, {255, 204, 204}, {204, 204, 0}};
   for (int i = 0 ; i < 100 ; i++)
   {
       int idxI = i%10;
       int posI = idxI*100;
       int idxJ = i/10;
       int posJ = idxJ*100;
       int firstPt = (i%3);
       rv[i].X[firstPt] = posI;
       if (i == 50)
           rv[i].X[firstPt] = -10;
       rv[i].Y[firstPt] = posJ;
       rv[i].X[(firstPt+1)%3] = posI+99;
       rv[i].Y[(firstPt+1)%3] = posJ;
       rv[i].X[(firstPt+2)%3] = posI+i;
       rv[i].Y[(firstPt+2)%3] = posJ+10*(idxJ+1);
       if (i == 95)
          rv[i].Y[(firstPt+2)%3] = 1050;
       rv[i].color[0] = colors[i%6][0];
       rv[i].color[1] = colors[i%6][1];
       rv[i].color[2] = colors[i%6][2];
   }

   return rv;
}

void writeTriangle(Triangle triangle, unsigned char *buffer){
  double bottomLeftX, bottomLeftY, bottomRightX, bottomRightY, topX, topY;
  if(triangle.Y[0] == triangle.Y[1]){
    bottomRightY = triangle.Y[0];
    bottomLeftY  = triangle.Y[0];
    if(triangle.X[0] < triangle.X[1]){
      bottomLeftX  = triangle.X[0];
      bottomRightX = triangle.X[1];
    }
    else{
      bottomLeftX  = triangle.X[1];
      bottomRightX = triangle.X[0];
    }
    topX = triangle.X[2];
    topY = triangle.Y[2];
  }
  else if(triangle.Y[0] == triangle.Y[2]){
    bottomRightY = triangle.Y[0];
    bottomLeftY  = triangle.Y[0];
    if(triangle.X[0] < triangle.X[2]){
      bottomLeftX  = triangle.X[0];
      bottomRightX = triangle.X[2];
    }
    else{
      bottomLeftX  = triangle.X[2];
      bottomRightX = triangle.X[0];
    }
    topX = triangle.X[1];
    topY = triangle.Y[1];
  }
  else{
    bottomRightY = triangle.Y[1];
    bottomLeftY  = triangle.Y[1];
    if(triangle.X[1] < triangle.X[2]){
      bottomLeftX  = triangle.X[1];
      bottomRightX = triangle.X[2];
    }
    else{
      bottomLeftX  = triangle.X[2];
      bottomRightX = triangle.X[1];
    }
    topX = triangle.X[0];
    topY = triangle.Y[0];
  }

  int ymin, ymax;
  if(bottomLeftY < 0){
    ymin = 0;
  }
  else{
    ymin = ceil441(bottomLeftY);
  }

  if(topY > 999){
    ymax = 999;
  }
  else{
    ymax = floor441(topY);
  }

  double* slopeLeft;
  double* slopeRight;

  if(topX == bottomLeftX){
    slopeLeft = NULL;
  }
  else{
    double val1 = (topY - bottomLeftY)/(topX - bottomLeftX);
    slopeLeft = &val1;
  }
  if(topX == bottomRightX){
    slopeRight = NULL;
  }
  else{
    double val2 = (bottomRightY - topY)/(bottomRightX - topX);
    slopeRight = &val2;
  }


  for(int i = ymin; i <= ymax; i++){
    int rightBound, leftBound, currentLeftX, currentRightX;
    if(slopeLeft != NULL){
      currentLeftX = ceil441(bottomLeftX + (((double) i) - ymin)/(*slopeLeft));
    }
    else{
      currentLeftX = ceil441(bottomLeftX);
    }

    if(currentLeftX < 0){
      leftBound = 0;
    }
    else{
      leftBound = currentLeftX;
    }

    if(slopeRight != NULL){
      currentRightX = bottomRightX + (((double) i) - ymin)/(*slopeRight);
    }
    else{
      currentRightX = bottomRightX;
    }

    if(currentRightX > 999){
      rightBound = 999;
    }
    else{
      rightBound = currentRightX;
    }

    for(int j = leftBound; j <= rightBound; j++){
      int currentPixel = 1000*i + j;
      int currentIndex = 3*currentPixel;
      buffer[currentIndex] = triangle.color[0];
      buffer[currentIndex + 1] = triangle.color[1];
      buffer[currentIndex + 2] = triangle.color[2];
    }
  }

}

int main()
{
   vtkImageData *image = NewImage(1000, 1000);
   unsigned char *buffer = 
     (unsigned char *) image->GetScalarPointer(0,0,0);
   int npixels = 1000*1000;
   for (int i = 0 ; i < npixels*3 ; i++)
       buffer[i] = 0;

   std::vector<Triangle> triangles = GetTriangles();
   
   Screen screen;
   screen.buffer = buffer;
   screen.width = 1000;
   screen.height = 1000;

   double minX, minY, midX, midY, maxX, maxY;

   //for each triangle
   for(int j = 0; j<100; j++){
    Triangle currentTriangle = triangles[j];
    
    writeTriangle(currentTriangle, buffer);



    
   }
   printf("finished writing triangles to buffer\n");

   WriteImage(image, "project1Bfinished");
}


