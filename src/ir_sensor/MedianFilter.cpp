#include "MedianFilter.h"

template<class T>
const T& constrain(const T& x, const T& a, const T& b) {
    if(x < a) {
        return a;
    }
    else if(b < x) {
        return b;
    }
    else
        return x;
}

MedianFilter::MedianFilter(int size, int seed)
{
   medFilterWin    = constrain(size, 3, 255); // number of samples in sliding median filter window - usually odd #
   medDataPointer  = size >> 1;           // mid point of window
   data            = (int*)     calloc (size, sizeof(int));     // array for data
   sizeMap         = (int*) calloc (size, sizeof(int)); // array for locations of data in sorted list
   locationMap     = (int*) calloc (size, sizeof(int)); // array for locations of history data in map list
   oldestDataPoint = medDataPointer;      // oldest data point location in data array
   totalSum        = size * seed;         // total of all values

   for(int i = 0; i < medFilterWin; i++) // initialize the arrays
   {
      sizeMap[i]     = i;      // start map with straight run
      locationMap[i] = i;      // start map with straight run
      data[i]        = seed;   // populate with seed value
   }
}

MedianFilter::~MedianFilter()
{
  // Free up the used memory when the object is destroyed
  free(data);
  free(sizeMap);
  free(locationMap);
}

int MedianFilter::in(const int & value)
{
   // sort sizeMap
   // small vaues on the left (-)
   // larger values on the right (+)

   bool dataMoved = false;
   const int rightEdge = medFilterWin - 1;  // adjusted for zero indexed array

   totalSum += value - data[oldestDataPoint];  // add new value and remove oldest value

   data[oldestDataPoint] = value;  // store new data in location of oldest data in ring buffer

   // SORT LEFT (-) <======(n) (+)
   if(locationMap[oldestDataPoint] > 0) // don't check left neighbours if at the extreme left
   {
      for(int i = locationMap[oldestDataPoint]; i > 0; i--)   //index through left adjacent data
      {
         int n = i - 1;   // neighbour location

         if(data[oldestDataPoint] < data[sizeMap[n]]) // find insertion point, move old data into position
         {
            sizeMap[i] = sizeMap[n];   // move existing data right so the new data can go left
            locationMap[sizeMap[n]]++;

            sizeMap[n] = oldestDataPoint; // assign new data to neighbor position
            locationMap[oldestDataPoint]--;

            dataMoved = true;
         }
         else
         {
            break; // stop checking once a smaller value is found on the left
         }
      }
   }

   // SORT RIGHT (-) (n)======> (+)
   if(!dataMoved && locationMap[oldestDataPoint] < rightEdge) // don't check right if at right border, or the data has already moved
   {
      for(int i = locationMap[oldestDataPoint]; i < rightEdge; i++)   //index through left adjacent data
      {
         int n = i + 1;   // neighbour location

         if(data[oldestDataPoint] > data[sizeMap[n]]) // find insertion point, move old data into position
         {
            sizeMap[i] = sizeMap[n];   // move existing data left so the new data can go right
            locationMap[sizeMap[n]]--;

            sizeMap[n] = oldestDataPoint; // assign new data to neighbor position
            locationMap[oldestDataPoint]++;
         }
         else
         {
            break; // stop checking once a smaller value is found on the right
         }
      }
   }
   oldestDataPoint++;       // increment and wrap
   if(oldestDataPoint == medFilterWin) oldestDataPoint = 0;

   return data[sizeMap[medDataPointer]];
}


int MedianFilter::out() // return the value of the median data sample
{
   return  data[sizeMap[medDataPointer]];
}


int MedianFilter::getMin()
{
   return data[sizeMap[ 0 ]];
}


int MedianFilter::getMax()
{
   return data[sizeMap[ medFilterWin - 1 ]];
}


int MedianFilter::getMean()
{
   return totalSum / medFilterWin;
}


int MedianFilter::getStDev()  // Arduino run time [us]: filterSize * 2 + 131
{
   int32_t diffSquareSum = 0;
   int mean = getMean();

   for( int i = 0; i < medFilterWin; i++ )
   {
      int diff = data[i] - mean;
      diffSquareSum += diff * diff;
   }

   return int( sqrtf( float(diffSquareSum) / float(medFilterWin - 1) ) + 0.5f );
}