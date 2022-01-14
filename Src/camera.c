/*
 * camera.c
 *
 *  Created on: Nov 3, 2018
 *      Author: Graham Thoms
 */

#include "camera.h"
#include "arducam.h"

#include "malloc.h"
#include "stlogo.h"
#include "stm32f429i_discovery_lcd.h"
#include "usbd_cdc_if.h"
#include "picojpeg.h"

#ifndef MAX_JPEG_SIZE
#define MAX_JPEG_SIZE 40000
#endif

static uint32_t captureStart;
uint8_t fifoBuffer[BURST_READ_LENGTH];

uint8_t ptr_picture[MAX_JPEG_SIZE] = {0};
uint32_t picture_length;


static void camera_get_image();
BaseType_t write_fifo_to_buffer(uint32_t length);
static void process_picture();

//Added functions
#ifndef min
#define min(a,b)    (((a) < (b)) ? (a) : (b))
#endif
unsigned char pjpeg_need_bytes_callback(unsigned char* pBuf, unsigned char buf_size,
										unsigned char *pBytes_actually_read, void *pCallback_data);
uint8_t *pjpeg_load_from_stream(uint8_t pStream[MAX_JPEG_SIZE], int *x, int *y, int *comps, pjpeg_scan_type_t *pScan_type, int reduce);

static FILE *g_pInFile;
static uint g_nInFileSize;
static uint g_nInFileOfs;

extern int sensor_flag;
static void display_bitmap(const uint8_t * image, int width, int height, int comps, int start_x, int start_y, int scale);
static void get_pixel(int* pDst, const uint8_t *pSrc, int luma_only, int num_comps);
//End of added functions

void camera_setup(){

	cameraReady = pdFALSE;
	/**
	 * Detect and initialize the Arduchip interface.
	 * Ensure that the OV5642 is powered on.
	 * Detect and initialize the OV5642 sensor chip.
	 */
	if (   arduchip_detect()
		&& arducam_exit_standby()
		&& ov5642_detect()
	) {

		osDelay(100);

		if (!ov5642_configure()) {
			printf("camera_task: ov5642 configure failed\n\r");
			return;
		} else {
			printf("camera: setup complete\n\r");
			cameraReady = pdTRUE;
			osDelay(100);
		}
	} else {
		printf("camera: setup failed\n\r");
		cameraReady = pdTRUE;
	}
}

/**
 * Capture an image from the camera.
 */
void camera_initiate_capture(){

	uint8_t done = 0;

	printf("camera: initiate capture\n\r");

	if (!cameraReady) {
		printf("camera: set up camera before capture\n\r");
	}else if(arducam_exit_standby()){
		printf("camera: exiting standby mode\n\r");
		osDelay(1000);
	}

	/* Initiate an image capture. */
	if (!arduchip_start_capture()) {
		printf("camera: initiate capture failed\n\r");
		return;
	}

	/* wait for capture to be done */
	captureStart = (uint32_t)xTaskGetTickCount();
	while(!arduchip_capture_done(&done) || !done){

		if ((xTaskGetTickCount() - captureStart) >= CAPTURE_TIMEOUT) {
			printf("camera: capture timeout\n\r");
			return;
		}
	}

	printf("camera: capture complete\n\r");
	osDelay(500);

	camera_get_image();

	arducam_enter_standby();
	printf("camera: on standby\n\r");
	return;

}

void camera_get_image(){

	/* Determine the FIFO buffer length. */
	uint32_t length = 0;
	if (arduchip_fifo_length(&length) == pdTRUE) {
		printf("camera: captured jpeg image -> %lu bytes\n\r", length);
		picture_length = length;
		if(picture_length > 0)
			write_fifo_to_buffer(length);

	} else {
		printf("camera: get fifo length failed\n\r");
	}

	return;
}

BaseType_t
write_fifo_to_buffer(uint32_t length) {
	/* Write the FIFO contents to disk. */
	uint16_t chunk = 0;
	uint32_t pic_index = 0;

	memset(ptr_picture, 0, sizeof(ptr_picture));
	pic_index = 0;

	for(uint16_t i = 0; length > 0; i++)
	{
		chunk = MIN(length, BURST_READ_LENGTH);
		arduchip_burst_read(fifoBuffer, chunk);

		for(uint16_t j = 0; j < chunk; j++) //copy contents of chunk to ptr_picture
		{
			ptr_picture[pic_index] = fifoBuffer[j];
			pic_index++;
		}

		length -= chunk;
	}

	printf("jpeg test %x, %x, %x\r\n", ptr_picture[0], ptr_picture[1], ptr_picture[2]);//To check for JPEG SOI (ff d8 ff)

	process_picture();

	// test image: make sure to build the project with -Og to show this static .bmp image
	// 		project properties -> C/C++ Build -> Settings -> Optimization | Optimize for debugging (-Og)
    //BSP_LCD_DrawBitmap(80, 180, (uint8_t *)stlogo);

    osDelay(500);

	return pdTRUE;
}

static void process_picture(){
	int width, height, comps;
	pjpeg_scan_type_t scan_type;
	uint8_t *pImage;
	int reduce = 1;

	pImage = pjpeg_load_from_stream(ptr_picture, &width, &height, &comps, &scan_type, reduce);
	if(pImage == NULL)
	{
		printf("pImage failed\r\n");
	}
	else
	{
	  printf("pImage success\r\n");
	  BSP_LCD_Clear(LCD_COLOR_WHITE);
	  if(sensor_flag){
		  display_bitmap(pImage, width, height, comps, 0, 0, 240/width); //0 deg
	  }else{
		  display_bitmap(pImage, width, height, comps, 320, 240, 320/width); //90 deg CW
	  }

	  free(pImage);
	}

	return;
}


static void get_pixel(int* pDst, const uint8_t *pSrc, int luma_only, int num_comps)
{
   int r, g, b;
   if (num_comps == 1)
   {
      r = g = b = pSrc[0];
   }
   else if (luma_only)
   {
      const int YR = 19595, YG = 38470, YB = 7471;
      r = g = b = (pSrc[0] * YR + pSrc[1] * YG + pSrc[2] * YB + 32768) / 65536;
   }
   else
   {
      r = pSrc[0]; g = pSrc[1]; b = pSrc[2];
   }
   pDst[0] = r; pDst[1] = g; pDst[2] = b;
}

//Returned image can be rotated by 90 deg CW to display on entire board if call and drawpixel lines are changed to such.
static void display_bitmap(const uint8_t * image, int width, int height, int comps, int start_x, int start_y, int scale)
{
	int pSrc[3];
	uint32_t argb;
	int x, y;
	int scale_x, scale_y;

	for(y = 0; y < height; y++)
	{
		for (x = 0; x < width; x++)
		{
			get_pixel(pSrc, image + (y * width + x) * comps, 0, comps);
			argb = (0xFF << 24) | (pSrc[0] << 16) | (pSrc[1] << 8) | (pSrc[2]); //alpha (opacity), red, green, blue (8 bits each, 32 bit colour)

			for (scale_y = 0; scale_y < scale; scale_y++)
			{
				for (scale_x = 0; scale_x < scale; scale_x++)
				{
					if(sensor_flag){
						BSP_LCD_DrawPixel(start_x + (x * scale + scale_x), start_y + (y * scale + scale_y), argb); //0 deg
					}else{
						BSP_LCD_DrawPixel(start_y - (y * scale + scale_y), start_x - (x * scale + scale_x), argb); //90 deg CW
					}
				}
			}
		}
	}
}

//Helper functions pulled from fpg2tga.c and modified for memory buffer stream

unsigned char pjpeg_need_bytes_callback(unsigned char* pBuf, unsigned char buf_size, unsigned char *pBytes_actually_read, void *pCallback_data)
{
   uint8_t n;
//   pCallback_data;

   n = min(g_nInFileSize - g_nInFileOfs, buf_size);
   if (n && (fread(pBuf, 1, n, g_pInFile) != n))
      return PJPG_STREAM_READ_ERROR;
   *pBytes_actually_read = (unsigned char)(n);
   g_nInFileOfs += n;
   return 0;
}

uint8_t *pjpeg_load_from_stream(uint8_t pStream[MAX_JPEG_SIZE], int *x, int *y, int *comps, pjpeg_scan_type_t *pScan_type, int reduce)
{
   pjpeg_image_info_t image_info;
   int mcu_x = 0;
   int mcu_y = 0;
   uint row_pitch;
   uint8_t *pImage;
   uint8_t status;
   uint decoded_width, decoded_height;
   uint row_blocks_per_mcu, col_blocks_per_mcu;

   *x = 0;
   *y = 0;
   *comps = 0;
   if (pScan_type) *pScan_type = PJPG_YH1V1;

   g_pInFile = fmemopen(pStream, picture_length, "rb");//Modified for memory stream
   if (!g_pInFile)
      return NULL;

   g_nInFileOfs = 0;

   fseek(g_pInFile, 0, SEEK_END);
   g_nInFileSize = ftell(g_pInFile);
   fseek(g_pInFile, 0, SEEK_SET);
   //TODO: Find EOI/EOF via char check instead of the above. Above still lets the image work, but might end the image much earlier than expected.
   //Expression to use: "pStream[g_nInFileSize-1]"
   status = pjpeg_decode_init(&image_info, pjpeg_need_bytes_callback, NULL, (unsigned char)reduce);

   if (status)
   {
      printf("pjpeg_decode_init() failed with status %u\r\n", status);
      if (status == PJPG_UNSUPPORTED_MODE)
      {
         printf("Progressive JPEG files are not supported.\r\n");
      }

      if (status == PJPG_NOT_JPEG)
      {
         printf("Stream provided is not a JPEG.\r\n");
      }


      fclose(g_pInFile);
      return NULL;
   }

   if (pScan_type)
      *pScan_type = image_info.m_scanType;

   // In reduce mode output 1 pixel per 8x8 block.
   decoded_width = reduce ? (image_info.m_MCUSPerRow * image_info.m_MCUWidth) / 8 : image_info.m_width;
   decoded_height = reduce ? (image_info.m_MCUSPerCol * image_info.m_MCUHeight) / 8 : image_info.m_height;

   row_pitch = decoded_width * image_info.m_comps;
   pImage = (uint8_t *)malloc(row_pitch * decoded_height);
   if (!pImage)
   {
      fclose(g_pInFile);
      return NULL;
   }

   row_blocks_per_mcu = image_info.m_MCUWidth >> 3;
   col_blocks_per_mcu = image_info.m_MCUHeight >> 3;

   for ( ; ; )
   {
      int y, x;
      uint8_t *pDst_row;

      status = pjpeg_decode_mcu();

      if (status)
      {
         if (status != PJPG_NO_MORE_BLOCKS)
         {
            printf("pjpeg_decode_mcu() failed with status %u\n", status);

            free(pImage);
            fclose(g_pInFile);
            return NULL;
         }

         break;//Where the function finishes decompression in this loop.
      }

      if (mcu_y >= image_info.m_MCUSPerCol)
      {
         free(pImage);
         fclose(g_pInFile);
         return NULL;
      }

      if (reduce)
      {
         // In reduce mode, only the first pixel of each 8x8 block is valid.
         pDst_row = pImage + mcu_y * col_blocks_per_mcu * row_pitch + mcu_x * row_blocks_per_mcu * image_info.m_comps;
         if (image_info.m_scanType == PJPG_GRAYSCALE)
         {
            *pDst_row = image_info.m_pMCUBufR[0];
         }
         else
         {
            uint y, x;
            for (y = 0; y < col_blocks_per_mcu; y++)
            {
               uint src_ofs = (y * 128U);
               for (x = 0; x < row_blocks_per_mcu; x++)
               {
                  pDst_row[0] = image_info.m_pMCUBufR[src_ofs];
                  pDst_row[1] = image_info.m_pMCUBufG[src_ofs];
                  pDst_row[2] = image_info.m_pMCUBufB[src_ofs];
                  pDst_row += 3;
                  src_ofs += 64;
               }

               pDst_row += row_pitch - 3 * row_blocks_per_mcu;
            }
         }
      }
      else
      {
         // Copy MCU's pixel blocks into the destination bitmap.
         pDst_row = pImage + (mcu_y * image_info.m_MCUHeight) * row_pitch + (mcu_x * image_info.m_MCUWidth * image_info.m_comps);

         for (y = 0; y < image_info.m_MCUHeight; y += 8)
         {
            const int by_limit = min(8, image_info.m_height - (mcu_y * image_info.m_MCUHeight + y));

            for (x = 0; x < image_info.m_MCUWidth; x += 8)
            {
               uint8_t *pDst_block = pDst_row + x * image_info.m_comps;

               // Compute source byte offset of the block in the decoder's MCU buffer.
               uint src_ofs = (x * 8U) + (y * 16U);
               const uint8_t *pSrcR = image_info.m_pMCUBufR + src_ofs;
               const uint8_t *pSrcG = image_info.m_pMCUBufG + src_ofs;
               const uint8_t *pSrcB = image_info.m_pMCUBufB + src_ofs;

               const int bx_limit = min(8, image_info.m_width - (mcu_x * image_info.m_MCUWidth + x));

               if (image_info.m_scanType == PJPG_GRAYSCALE)
               {
                  int bx, by;
                  for (by = 0; by < by_limit; by++)
                  {
                     uint8_t *pDst = pDst_block;

                     for (bx = 0; bx < bx_limit; bx++)
                        *pDst++ = *pSrcR++;

                     pSrcR += (8 - bx_limit);

                     pDst_block += row_pitch;
                  }
               }
               else
               {
                  int bx, by;
                  for (by = 0; by < by_limit; by++)
                  {
                     uint8_t *pDst = pDst_block;

                     for (bx = 0; bx < bx_limit; bx++)
                     {
                        pDst[0] = *pSrcR++;
                        pDst[1] = *pSrcG++;
                        pDst[2] = *pSrcB++;
                        pDst += 3;
                     }

                     pSrcR += (8 - bx_limit);
                     pSrcG += (8 - bx_limit);
                     pSrcB += (8 - bx_limit);

                     pDst_block += row_pitch;
                  }
               }
            }

            pDst_row += (row_pitch * 8);
         }
      }

      mcu_x++;
      if (mcu_x == image_info.m_MCUSPerRow)
      {
         mcu_x = 0;
         mcu_y++;
      }
   }

   fclose(g_pInFile);

   *x = decoded_width;
   *y = decoded_height;
   *comps = image_info.m_comps;

   return pImage;
}

