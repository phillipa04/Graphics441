#include <iostream>
#include <vtkDataSet.h>
#include <vtkImageData.h>
#include <vtkPNGWriter.h>
#include <vtkPointData.h>

using std::cerr;
using std::endl;

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


int main()
{
   std::cerr << "In main!" << endl;
   vtkImageData *image = NewImage(1024, 1350);
   unsigned char *buffer = 
     (unsigned char *) image->GetScalarPointer(0,0,0);

     //1024 pixels per row and 50 rows per strip
   int numPixelsPerStrip = 1024*50;
   int numChannelsPerStrip = numPixelsPerStrip*3;

   //initialize rbg values
   int red = 0;
   int blue = 0;
   int green = 0;

   //loop through the strips and determine the rules for each strip based on assignment
   for(int i=0; i<27; i++){
    int blueInt = i%3;
    int greenInt = (i/3)%3;
    int redInt = i/9;

    //determine color components based on rules from assignment

    //determine blue channel
    if(blueInt == 0){
      blue = 0;
    }
    else if(blueInt == 1){
      blue = 128;
    }
    else{
      blue = 255;
    }

    //determine green channel
    if(greenInt == 0){
      green = 0;
    }
    else if(greenInt == 1){
      green = 128;
    }
    else{
      green = 255;
    }

    //determine red channel
    if(redInt == 0){
      red = 0;
    }
    else if(redInt == 1){
      red = 128;
    }
    else{
      red = 255;
    }

    //write the pixels to the buffer for the current strip
    for(int j = 0; j<numChannelsPerStrip; j+=3){
      buffer[i*numChannelsPerStrip + j] = red;
      buffer[i*numChannelsPerStrip + j + 1] = green;
      buffer[i*numChannelsPerStrip + j + 2] = blue;
    }
   }
   WriteImage(image, "finishedAssignment1A");
}
