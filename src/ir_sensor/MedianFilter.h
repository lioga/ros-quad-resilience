
#ifndef MedianFilter_h

   #define MedianFilter_h
    
   #include <cstdlib>
   #include <cmath> 
   class MedianFilter
   {
      public:
         MedianFilter(int size, int seed);
         ~MedianFilter();
         int in(const int & value);
         int out();

         int getMin();
         int getMax();
         int getMean();
         int getStDev();

         /*
         void printData();		// used for debugging
         void printSizeMap();
         void printLocationMap();
         void printSortedData();
         */

      private:
         int medFilterWin;      // number of samples in sliding median filter window - usually odd #
         int medDataPointer;	   // mid point of window
         int     * data;			   // array pointer for data sorted by age in ring buffer
         int * sizeMap;			// array pointer for locations data in sorted by size
         int * locationMap;		// array pointer for data locations in history map
         int oldestDataPoint;	// oldest data point location in ring buffer
         int totalSum;
   };

#endif