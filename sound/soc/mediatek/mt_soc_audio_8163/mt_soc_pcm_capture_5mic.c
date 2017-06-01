/*
 * Copyright (C) 2007 The Android Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
/*******************************************************************************
 *
 * Filename:
 * ---------
 *   mtk_pcm_capture.c
 *
 * Project:
 * --------
 *   Audio Driver Kernel Function
 *
 * Description:
 * ------------
 *   Audio Ul1 data1 uplink
 *
 * Author:
 * -------
 * Chipeng Chang
 *
 *------------------------------------------------------------------------------
 * $Revision: #1 $
 * $Modtime:$
 * $Log:$
 *
 *
 *******************************************************************************/


/*****************************************************************************
 *                     C O M P I L E R   F L A G S
 *****************************************************************************/


/*****************************************************************************
 *                E X T E R N A L   R E F E R E N C E S
 *****************************************************************************/

#include <linux/dma-mapping.h>
#include "AudDrv_Common.h"
#include "AudDrv_Def.h"
#include "AudDrv_Afe.h"
#include "AudDrv_Ana.h"
#include "AudDrv_Clk.h"
#include "AudDrv_Kernel.h"
#include "mt_soc_afe_control.h"
//#define AUDIO_ALLOCATE_SMP_RATE_DECLARE
#include "mt_soc_pcm_common.h"

#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#include <linux/wakelock.h>
#endif

//#define CAPTURE_FORCE_USE_DRAM //foruse DRAM for record

//information about
static AFE_MEM_CONTROL_T  *VUL_Control_context;
static struct snd_dma_buffer *Capture_dma_buf  = NULL;
//static struct snd_dma_buffer *AWBCapture_dma_buf  = NULL;
//static struct snd_dma_buffer *VULCapture_dma_buf  = NULL;
static AudioDigtalI2S *mAudioDigitalI2S = NULL;
static bool mCaptureUseSram = false;
static DEFINE_SPINLOCK(auddrv_ULInCtl_lock);
#define Samp_96 96000
static unsigned int GPIO_RST_EN;
static unsigned int GPIO_89;
static unsigned int GPIO_5;
static unsigned int GPIO_42;
static struct wake_lock m_lock;

static void fpga_set_gpio_output(unsigned int GPIO, unsigned int output);

/*
 *    function implementation
 */
static void StartAudioCaptureHardware(struct snd_pcm_substream *substream);
static void StopAudioCaptureHardware(struct snd_pcm_substream *substream);
void StartAudioCaptureAnalogHardware(void);
void StopAudioCaptureAnalogHardware(void);
static int mtk_capture_probe(struct platform_device *pdev);
static int mtk_capture_pcm_close(struct snd_pcm_substream *substream);
static int mtk_asoc_capture_pcm_new(struct snd_soc_pcm_runtime *rtd);
static int mtk_afe_capture_probe(struct snd_soc_platform *platform);

static struct snd_pcm_hardware mtk_capture_hardware =
{
    .info = (SNDRV_PCM_INFO_MMAP |SNDRV_PCM_INFO_BLOCK_TRANSFER |
    SNDRV_PCM_INFO_INTERLEAVED |
    SNDRV_PCM_INFO_RESUME |
    SNDRV_PCM_INFO_MMAP_VALID),
    .formats =      SND_SOC_ADV_MT_FMTS,
    .rates =        SOC_HIGH_USE_RATE,
    .rate_min =     SOC_HIGH_USE_RATE_MIN,
    .rate_max =     SOC_HIGH_USE_RATE_MAX,
    .channels_min =     SOC_NORMAL_USE_CHANNELS_MIN,
    .channels_max =     SOC_NORMAL_USE_CHANNELS_MAX,
    .buffer_bytes_max = UL1_MAX_BUFFER_SIZE,//AWB_MAX_BUFFER_SIZE,//UL1_MAX_BUFFER_SIZE,//AWB_MAX_BUFFER_SIZE,//
    .period_bytes_max = UL1_MAX_BUFFER_SIZE,//AWB_MAX_BUFFER_SIZE,//AWB_MAX_BUFFER_SIZE,//UL1_MAX_BUFFER_SIZE,
    .periods_min =  UL1_MIN_PERIOD_SIZE,//AWB_MIN_PERIOD_SIZE,   //UL1_MIN_PERIOD_SIZE,
    .periods_max =  UL1_MAX_PERIOD_SIZE,// AWB_MAX_PERIOD_SIZE,   // UL1_MAX_PERIOD_SIZE,
    .fifo_size =      0,
};

static void StopAudioCaptureHardware(struct snd_pcm_substream *substream)
{
    printk("5mic--->StopAudioCaptureHardware \n");

    // here to set interrupt
	
	if(substream->runtime->rate == Samp_96)
	{

		    SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_2, false);
		    if (GetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_2) == false)
		    {
				DisableALLbySampleRate(Samp_96);
				
				printk("5mic--->StopAudioCaptureHardware APLL disable\n");

		//EnableI2SDivPower(AUDIO_APLL1_DIV0, false); 
		//EnableI2SDivPower(AUDIO_APLL2_DIV0, false);
		EnableI2SDivPower(AUDIO_APLL12_DIV1, false); 
		//EnableI2SDivPower(AUDIO_APLL12_DIV2 , false); 
		//EnableI2SDivPower(AUDIO_APLL12_DIV3 , false);
		//EnableI2SDivPower(AUDIO_APLL12_DIV4 , false);

				Set2ndI2SInEnable(false);	
		    }

		    SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_AWB, false);
		    SetIrqEnable(Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE, false);

		    // here to turn off digital part
		    SetConnection(Soc_Aud_InterCon_DisConnect, Soc_Aud_InterConnectionInput_I00, Soc_Aud_InterConnectionOutput_O05);
		    SetConnection(Soc_Aud_InterCon_DisConnect, Soc_Aud_InterConnectionInput_I01, Soc_Aud_InterConnectionOutput_O06);
	}
	else
	{
		SetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_ADC, false);
		   if (GetMemoryPathEnable(Soc_Aud_Digital_Block_I2S_IN_ADC) == false)
		   {
			   SetI2SAdcEnable(false);
		   }
		
		   SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_VUL, false);
		
		   SetIrqEnable(Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE, false);
		
		   // here to turn off digital part
		   SetConnection(Soc_Aud_InterCon_DisConnect, Soc_Aud_InterConnectionInput_I03, Soc_Aud_InterConnectionOutput_O09);
		   SetConnection(Soc_Aud_InterCon_DisConnect, Soc_Aud_InterConnectionInput_I04, Soc_Aud_InterConnectionOutput_O10);

	}
    EnableAfe(false);
}

static void ConfigAdcI2S(struct snd_pcm_substream *substream)
{
    mAudioDigitalI2S->mLR_SWAP = Soc_Aud_LR_SWAP_NO_SWAP;
    mAudioDigitalI2S->mBuffer_Update_word = 8;
    mAudioDigitalI2S->mFpga_bit_test = 0;
    mAudioDigitalI2S->mFpga_bit = 0;
    mAudioDigitalI2S->mloopback = 0;
    mAudioDigitalI2S->mINV_LRCK = Soc_Aud_INV_LRCK_NO_INVERSE;
    mAudioDigitalI2S->mI2S_FMT = Soc_Aud_I2S_FORMAT_I2S;
    mAudioDigitalI2S->mI2S_WLEN = Soc_Aud_I2S_WLEN_WLEN_16BITS;
    mAudioDigitalI2S->mI2S_SAMPLERATE = (substream->runtime->rate);
}

static void StartAudioCaptureHardware(struct snd_pcm_substream *substream)
{

    
	
	AudioDigtalI2S      m2ndI2SInAttribute;
	//AudioDigitalPCM p_modem_pcm_attribute;
	//uint32 reg_pcm_intf_con1 = 0;
	//uint32 InterfaceType, eFetchFormat;
	//uint32 ConnectionFormat;
	uint32 Aud_block,MclkDiv0;
	printk("5mic--->StartAudioCaptureHardware  substream->name=%s ,=%d\n",substream->name,substream->runtime->rate);

      if(substream->runtime->rate == Samp_96)
      	{
	     SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_AWB, AFE_WLEN_32_BIT_ALIGN_24BIT_DATA_8BIT_0);
	    SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_AWB, AFE_WLEN_32_BIT_ALIGN_24BIT_DATA_8BIT_0);
	    SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_24BIT, Soc_Aud_InterConnectionOutput_O05);
	    SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_24BIT, Soc_Aud_InterConnectionOutput_O06);
		Aud_block=Soc_Aud_Digital_Block_I2S_IN_2;
		// Config 2nd I2S IN
		 memset((void *)&m2ndI2SInAttribute, 0, sizeof(m2ndI2SInAttribute));

	 }
	 else
	 {	
		 ConfigAdcI2S(substream);
		 SetI2SAdcIn(mAudioDigitalI2S);

		 SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_VUL, AFE_WLEN_16_BIT);
		SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_VUL, AFE_WLEN_16_BIT);
		SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_16BIT, Soc_Aud_InterConnectionOutput_O09);
		SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_16BIT, Soc_Aud_InterConnectionOutput_O10);
		Aud_block=Soc_Aud_Digital_Block_I2S_IN_ADC;

	 }
    
    if (GetMemoryPathEnable(Aud_block) == false)
    {
        SetMemoryPathEnable(Aud_block, true);        
         if(substream->runtime->rate == Samp_96)
      	{
        //m2ndI2SInAttribute.mLR_SWAP = Soc_Aud_LR_SWAP_LR_DATASWAP;//Soc_Aud_LR_SWAP_NO_SWAP;
	        m2ndI2SInAttribute.mI2S_IN_PAD_SEL =  Soc_Aud_I2S_IN_PAD_SEL_I2S_IN_FROM_IO_MUX; // I2S_IN_FROM_CONNSYS
	        m2ndI2SInAttribute.mI2S_SLAVE = Soc_Aud_I2S_SRC_MASTER_MODE;//Soc_Aud_I2S_SRC_SLAVE_MODE;
	        m2ndI2SInAttribute.mI2S_SAMPLERATE = Samp_96;//96000

	    m2ndI2SInAttribute.mINV_LRCK = Soc_Aud_INV_LRCK_NO_INVERSE;//Soc_Aud_INV_LRCK_INVESE_LRCK;//Soc_Aud_INV_LRCK_NO_INVERSE;
	    m2ndI2SInAttribute.mI2S_FMT = Soc_Aud_I2S_FORMAT_EIAJ;//Soc_Aud_I2S_FORMAT_EIAJ;//Soc_Aud_I2S_FORMAT_I2S;
	    m2ndI2SInAttribute.mI2S_WLEN = Soc_Aud_I2S_WLEN_WLEN_32BITS;//Soc_Aud_I2S_WLEN_WLEN_16BITS
	   m2ndI2SInAttribute.mI2S_HDEN =Soc_Aud_LOW_JITTER_CLOCK;

	   EnableALLbySampleRate(substream->runtime->rate);//enable clk;

	//   EnableI2SDivPower(AUDIO_APLL1_DIV0, true); 
	 //  EnableI2SDivPower(AUDIO_APLL2_DIV0, true);	   
	   EnableI2SDivPower(AUDIO_APLL12_DIV1, true); 
	  // EnableI2SDivPower(AUDIO_APLL12_DIV2 , true); 
	  // EnableI2SDivPower(AUDIO_APLL12_DIV3 , true);
	 //  EnableI2SDivPower(AUDIO_APLL12_DIV4 , true);
	   MclkDiv0 = SetCLkMclk(Soc_Aud_I2S0, m2ndI2SInAttribute.mI2S_SAMPLERATE); //select I2S
     SetCLkBclk(MclkDiv0,  m2ndI2SInAttribute.mI2S_SAMPLERATE, 2, Soc_Aud_I2S_WLEN_WLEN_32BITS); 
      Afe_Set_Reg(AUDIO_CLK_AUDDIV_0, 0x30000100, AFE_MASK_ALL);//
       Afe_Set_Reg(AUDIO_CLK_AUDDIV_1, 0x00000007, AFE_MASK_ALL); 
     
	   Set2ndI2SIn(&m2ndI2SInAttribute);
  
	    Set2ndI2SInEnable(true);
     	}
	 else
    	{
		 SetI2SAdcEnable(true);
	 }
		
    }
    else
    {
        SetMemoryPathEnable(Aud_block, true);
    }
	if(substream->runtime->rate == Samp_96)
	{

	   SetConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I00, Soc_Aud_InterConnectionOutput_O05);
	   SetConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I01, Soc_Aud_InterConnectionOutput_O06);
	}
	else
	{
		SetConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I03, Soc_Aud_InterConnectionOutput_O09);
		 SetConnection(Soc_Aud_InterCon_Connection, Soc_Aud_InterConnectionInput_I04, Soc_Aud_InterConnectionOutput_O10);
	}

    if (substream->runtime->format == SNDRV_PCM_FORMAT_S32_LE || substream->runtime->format == SNDRV_PCM_FORMAT_U32_LE)
    {
	    if(substream->runtime->rate == Samp_96)
		{
	      printk("orangeyang@20160303>>>>>>>>>>>>1111111>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>.\n");
	        SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_AWB, AFE_WLEN_32_BIT_ALIGN_24BIT_DATA_8BIT_0); //==3---> shift 1byte
	        SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_24BIT, Soc_Aud_InterConnectionOutput_O05);
	        SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_24BIT, Soc_Aud_InterConnectionOutput_O06);
	    	}
	 else
	 {
		 SetMemIfFetchFormatPerSample(Soc_Aud_Digital_Block_MEM_VUL, AFE_WLEN_32_BIT_ALIGN_8BIT_0_24BIT_DATA);
		SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_24BIT, Soc_Aud_InterConnectionOutput_O09);
		SetoutputConnectionFormat(OUTPUT_DATA_FORMAT_24BIT, Soc_Aud_InterConnectionOutput_O10); 

	 }
    }

    // here to set interrupt
     if(substream->runtime->rate == Samp_96)
   	 SetIrqMcuCounter(Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE, substream->runtime->period_size >> 1);//caution:
     else
	  SetIrqMcuCounter(Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE, substream->runtime->period_size );//caution:
    SetIrqMcuSampleRate(Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE, substream->runtime->rate);

  
    SetIrqEnable(Soc_Aud_IRQ_MCU_MODE_IRQ2_MCU_MODE, true);
    
     if(substream->runtime->rate == Samp_96)
     	{
	    SetSampleRate(Soc_Aud_Digital_Block_MEM_AWB, substream->runtime->rate);
	    SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_AWB, true);
	       SetChannels(Soc_Aud_Digital_Block_MEM_AWB, 2); //AWB_data stereo:0/mono:1
	      Afe_Set_Reg(AFE_DAC_CON0, (0x1<<6), (0x1<<6));  //AWB_ON    
     	}
	 else
	 {
		 SetSampleRate(Soc_Aud_Digital_Block_MEM_VUL, substream->runtime->rate);
		SetMemoryPathEnable(Soc_Aud_Digital_Block_MEM_VUL, true);
	 }
	// Afe_Set_Reg( AFE_CONN4, (0x1<<19)|(0x1<<20), (0x1<<19)|(0x1<<20));  //right shift 1bit
     // Afe_Set_Reg( AFE_CONN2, (0x1<<22)|(0x1<<16), (0x1<<22)|(0x1<<16));  //on/off  I0-->O5
    
    EnableAfe(true);
#ifdef DENALI_FPGA_EARLYPORTING //ccc early porting test, copy from TurnOnADcPowerACC()
	//here to set digital part
			   //Topck_Enable(true);
			   //AdcClockEnable(true);
			   //Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON1_L, 0x0000, 0xffff);	 //power on ADC clk //early porting K2 remove
	
			   Ana_Set_Reg(AFE_AUDIO_TOP_CON0, 0x0000, 0xffff);   //power on clock
			   //Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON1_L, 0x0000, 0xffff);	 //power on ADC clk //early porting K2 remove
			   Ana_Set_Reg(PMIC_AFE_TOP_CON0, 0x0000, 0xffff);	 //configure ADC setting
	
			   Ana_Set_Reg(AFE_UL_DL_CON0, 0x0001, 0xffff);   //turn on afe
	
			   Ana_Set_Reg(AFE_PMIC_NEWIF_CFG2, 0x302F, 0xffff); // config UL up8x_rxif adc voice mode, 8k sample rate
			   Ana_Set_Reg(AFE_UL_SRC0_CON0_H, (0 << 3 | 0 << 1) , 0x001f);// ULsampling rate, 8k sample rate
			   //Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON0_H, (ULSampleRateTransform(SampleRate_VUL2) << 3 | ULSampleRateTransform(SampleRate_VUL2) << 1) , 0x001f); // ULsampling rate
			   //Ana_Set_Reg(AFE_ADDA2_UL_SRC_CON0_L, 0x0041, 0xffff);
	
			   Ana_Set_Reg(AFE_UL_SRC0_CON0_L, 0x0005, 0xffff);   //power on uplink, and loopback to DL
	
			   Afe_Set_Reg(FPGA_CFG1, 0x1, 0xffff); // must set in FPGA platform for PMIC digital loopback
		   
#endif    

}

static int mtk_capture_pcm_prepare(struct snd_pcm_substream *substream)
{
    //printk("mtk_capture_pcm_prepare substream->rate = %d  substream->channels = %d \n", substream->runtime->rate, substream->runtime->channels);
    return 0;
}

static int mtk_capture_alsa_stop(struct snd_pcm_substream *substream)
{
    //AFE_BLOCK_T *Vul_Block = &(VUL_Control_context->rBlock);
    printk("5mic--->mtk_capture_alsa_stop \n");
    StopAudioCaptureHardware(substream);
	if(substream->runtime->rate == Samp_96)
  	  RemoveMemifSubStream(Soc_Aud_Digital_Block_MEM_AWB,substream);
	else
	 RemoveMemifSubStream(Soc_Aud_Digital_Block_MEM_VUL,substream);
    return 0;
}

static snd_pcm_uframes_t mtk_capture_pcm_pointer(struct snd_pcm_substream *substream)
{
    kal_int32 HW_memory_index = 0;
    kal_int32 HW_Cur_ReadIdx = 0;
    //kal_uint32 Frameidx = 0;
    kal_int32 Hw_Get_bytes = 0;
    bool bIsOverflow = false;
    unsigned long flags;
  uint32 Aud_block,offset;
    AFE_BLOCK_T *UL1_Block = &(VUL_Control_context->rBlock);
    PRINTK_AUD_UL1("5mic--->mtk_capture_pcm_pointer Awb_Block->u4WriteIdx;= 0x%x \n", UL1_Block->u4WriteIdx);
    Auddrv_UL1_Spinlock_lock();
    spin_lock_irqsave(&VUL_Control_context->substream_lock, flags);
	
	if(substream->runtime->rate == Samp_96)
	{
		Aud_block=Soc_Aud_Digital_Block_MEM_AWB;
		offset	=AFE_AWB_CUR;

	}
	else
	{
		Aud_block=Soc_Aud_Digital_Block_MEM_VUL;
		offset	=AFE_VUL_CUR;
	}
    if (GetMemoryPathEnable(Aud_block) == true)
    {

        HW_Cur_ReadIdx = Align64ByteSize(Afe_Get_Reg(offset));
        if (HW_Cur_ReadIdx == 0)
        {
            PRINTK_AUD_UL1("5mic--->[Auddrv] mtk_awb_pcm_pointer  HW_Cur_ReadIdx ==0 \n");
            HW_Cur_ReadIdx = UL1_Block->pucPhysBufAddr;
        }
        HW_memory_index = (HW_Cur_ReadIdx - UL1_Block->pucPhysBufAddr);

        // update for data get to hardware
        Hw_Get_bytes = (HW_Cur_ReadIdx - UL1_Block->pucPhysBufAddr) - UL1_Block->u4WriteIdx;
        if (Hw_Get_bytes < 0)
        {
            Hw_Get_bytes += UL1_Block->u4BufferSize;
        }
        UL1_Block->u4WriteIdx	+= Hw_Get_bytes;
        UL1_Block->u4WriteIdx	%= UL1_Block->u4BufferSize;
        UL1_Block->u4DataRemained += Hw_Get_bytes;
        

        // buffer overflow
        if (UL1_Block->u4DataRemained > UL1_Block->u4BufferSize)
        {
            bIsOverflow = true;
            printk("5mic--->mtk_capture_pcm_pointer buffer overflow u4DMAReadIdx:%x, u4WriteIdx:%x, u4DataRemained:%x, u4BufferSize:%x \n",
                   UL1_Block->u4DMAReadIdx, UL1_Block->u4WriteIdx, UL1_Block->u4DataRemained, UL1_Block->u4BufferSize);
        }
        
        PRINTK_AUD_UL1("5mic--->[Auddrv] mtk_capture_pcm_pointer =0x%x HW_memory_index = 0x%x\n", HW_Cur_ReadIdx, HW_memory_index);
        spin_unlock_irqrestore(&VUL_Control_context->substream_lock, flags);
        Auddrv_UL1_Spinlock_unlock();

        if (bIsOverflow == true)
        {
            return -1;
        }
        return audio_bytes_to_frame(substream, HW_memory_index);
    }
    spin_unlock_irqrestore(&VUL_Control_context->substream_lock, flags);
    Auddrv_UL1_Spinlock_unlock();
    return 0;

}

static void SetAWBBuffer(struct snd_pcm_substream *substream,
                         struct snd_pcm_hw_params *hw_params)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    AFE_BLOCK_T *pblock = &VUL_Control_context->rBlock;
    
    pblock->pucPhysBufAddr =  runtime->dma_addr;
    pblock->pucVirtBufAddr =  runtime->dma_area;
    pblock->u4BufferSize = runtime->dma_bytes;
    pblock->u4SampleNumMask = 0x001f;  // 32 byte align
    pblock->u4WriteIdx     = 0;
    pblock->u4DMAReadIdx    = 0;
    pblock->u4DataRemained  = 0;
    pblock->u4fsyncflag     = false;
    pblock->uResetFlag      = true;
    printk("5mic--->u4BufferSize = %d pucVirtBufAddr = %p pucPhysBufAddr = 0x%x\n",
           pblock->u4BufferSize, pblock->pucVirtBufAddr, pblock->pucPhysBufAddr);
    // set dram address top hardware
    Afe_Set_Reg(AFE_AWB_BASE , pblock->pucPhysBufAddr , 0xffffffff);
    Afe_Set_Reg(AFE_AWB_END  , pblock->pucPhysBufAddr + (pblock->u4BufferSize - 1), 0xffffffff);

}

static void SetVULBuffer(struct snd_pcm_substream *substream,
                         struct snd_pcm_hw_params *hw_params)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    AFE_BLOCK_T *pblock = &VUL_Control_context->rBlock;
    printk("5mic--->SetVULBuffer\n");
    pblock->pucPhysBufAddr =  runtime->dma_addr;
    pblock->pucVirtBufAddr =  runtime->dma_area;
    pblock->u4BufferSize = runtime->dma_bytes;
    pblock->u4SampleNumMask = 0x001f;  // 32 byte align
    pblock->u4WriteIdx     = 0;
    pblock->u4DMAReadIdx    = 0;
    pblock->u4DataRemained  = 0;
    pblock->u4fsyncflag     = false;
    pblock->uResetFlag      = true;
    printk("5mic--->u4BufferSize = %d pucVirtBufAddr = %p pucPhysBufAddr = 0x%x\n",
           pblock->u4BufferSize, pblock->pucVirtBufAddr, pblock->pucPhysBufAddr);
    // set dram address top hardware
    Afe_Set_Reg(AFE_VUL_BASE , pblock->pucPhysBufAddr , 0xffffffff);
    Afe_Set_Reg(AFE_VUL_END  , pblock->pucPhysBufAddr + (pblock->u4BufferSize - 1), 0xffffffff);

}

static int mtk_capture_pcm_hw_params(struct snd_pcm_substream *substream,
                                     struct snd_pcm_hw_params *hw_params)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    struct snd_dma_buffer *dma_buf = &substream->dma_buffer;
    int ret = 0;
    printk("5mic--->mtk_capture_pcm_hw_params rate=%d\n",substream->runtime->rate);

    dma_buf->dev.type = SNDRV_DMA_TYPE_DEV;
    dma_buf->dev.dev = substream->pcm->card->dev;
    dma_buf->private_data = NULL;

    if (mCaptureUseSram == true)
    {
        runtime->dma_bytes = params_buffer_bytes(hw_params);
        printk("5mic--->mtk_capture_pcm_hw_params mCaptureUseSram dma_bytes = %zu \n", runtime->dma_bytes);
        substream->runtime->dma_area = (unsigned char *)Get_Afe_SramBase_Pointer();
        substream->runtime->dma_addr = Get_Afe_Sram_Phys_Addr();
    }
    else if (Capture_dma_buf->area)
    {
        printk("5mic--->Capture_dma_buf = %p Capture_dma_buf->area = %p apture_dma_buf->addr = 0x%lx\n",
            Capture_dma_buf, Capture_dma_buf->area, (long) Capture_dma_buf->addr);
        runtime->dma_bytes = params_buffer_bytes(hw_params);
        runtime->dma_area = Capture_dma_buf->area;
        runtime->dma_addr = Capture_dma_buf->addr;
    }
    else
    {
        printk("5mic--->mtk_capture_pcm_hw_params snd_pcm_lib_malloc_pages\n");
        ret =  snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(hw_params));
    }
	if(substream->runtime->rate == Samp_96)
   	             SetAWBBuffer(substream, hw_params);
	else 
		 SetVULBuffer(substream, hw_params);
    printk("5mic--->dma_bytes = %zu dma_area = %p dma_addr = 0x%lx\n",
           substream->runtime->dma_bytes, substream->runtime->dma_area, (long)substream->runtime->dma_addr);
    return ret;
}

static int mtk_capture_pcm_hw_free(struct snd_pcm_substream *substream)
{
    printk("5mic--->mtk_capture_pcm_hw_free1 \n");
    if (Capture_dma_buf->area)
    {
        return 0;
    }
    else
    {
        return snd_pcm_lib_free_pages(substream);
    }
}

static struct snd_pcm_hw_constraint_list constraints_sample_rates =
{
    .count = ARRAY_SIZE(soc_high_supported_sample_rates),
   .list = soc_high_supported_sample_rates,
    
   // .count = ARRAY_SIZE(soc_normal_supported_sample_rates),
  //  .list = soc_normal_supported_sample_rates,
};

 //bool mic5_flag=false;  //@:mt8193_iic.c
static int mtk_capture_pcm_open(struct snd_pcm_substream *substream)
{
    struct snd_pcm_runtime *runtime = substream->runtime;
    int ret = 0;
//	printk("5mic--->mtk_capture_pcm_open rate==%d ---%s\n",substream->runtime->rate,__TIME__);

	//if(mic5_flag)
	    substream->runtime->rate = Samp_96;
	
	if(substream->runtime->rate == Samp_96)
	 {
	    AudDrv_ANA_Clk_On();    
	    AudDrv_I2S_Clk_On();
	    VUL_Control_context = Get_Mem_ControlT(Soc_Aud_Digital_Block_MEM_AWB);
	   Capture_dma_buf= Get_Mem_Buffer(Soc_Aud_Digital_Block_MEM_AWB);
	}
	else
	{
		AudDrv_ADC_Clk_On();
		 VUL_Control_context = Get_Mem_ControlT(Soc_Aud_Digital_Block_MEM_VUL);
		   Capture_dma_buf=Get_Mem_Buffer(Soc_Aud_Digital_Block_MEM_VUL);
	}
	AudDrv_Clk_On();

    // can allocate sram_dbg
    AfeControlSramLock();
	
	#ifndef CAPTURE_FORCE_USE_DRAM
    if ((substream->runtime->rate != Samp_96) && (GetSramState() ==  SRAM_STATE_FREE) )
	#else
    if (0)
    #endif
    {
        printk("5mic--->mtk_capture_pcm_open use sram \n");
       mtk_capture_hardware.buffer_bytes_max = GetCaptureSramSize();
        SetSramState(SRAM_STATE_CAPTURE);
        mCaptureUseSram = true;
    }
    else
    {
        printk("5mic--->mtk_capture_pcm_open use dram \n");
	if(substream->runtime->rate == Samp_96)	
           {
            mtk_capture_hardware.buffer_bytes_max = AWB_MAX_BUFFER_SIZE;//AWB_MAX_BUFFER_SIZE;// UL1_MAX_BUFFER_SIZE;	
	  mtk_capture_hardware.period_bytes_max = AWB_MAX_BUFFER_SIZE;//AWB_MAX_BUFFER_SIZE,//AWB_MAX_BUFFER_SIZE,//UL1_MAX_BUFFER_SIZE,
	  mtk_capture_hardware.periods_min =  AWB_MIN_PERIOD_SIZE;//AWB_MIN_PERIOD_SIZE,   //UL1_MIN_PERIOD_SIZE,
	  mtk_capture_hardware.periods_max =  AWB_MAX_PERIOD_SIZE;//

	}
	 else 
	 {
	     mtk_capture_hardware.buffer_bytes_max = UL1_MAX_BUFFER_SIZE;//AWB_MAX_BUFFER_SIZE,//UL1_MAX_BUFFER_SIZE,//AWB_MAX_BUFFER_SIZE,//
	    mtk_capture_hardware.period_bytes_max = UL1_MAX_BUFFER_SIZE;//AWB_MAX_BUFFER_SIZE,//AWB_MAX_BUFFER_SIZE,//UL1_MAX_BUFFER_SIZE,
	    mtk_capture_hardware.periods_min =  UL1_MIN_PERIOD_SIZE;//AWB_MIN_PERIOD_SIZE,   //UL1_MIN_PERIOD_SIZE,
	    mtk_capture_hardware.periods_max =  UL1_MAX_PERIOD_SIZE;//

	 }
    }
    AfeControlSramUnLock();

    runtime->hw = mtk_capture_hardware;
    memcpy((void *)(&(runtime->hw)), (void *)&mtk_capture_hardware , sizeof(struct snd_pcm_hardware));

   ret = snd_pcm_hw_constraint_list(runtime, 0, SNDRV_PCM_HW_PARAM_RATE,
                                   &constraints_sample_rates);
    ret = snd_pcm_hw_constraint_integer(runtime, SNDRV_PCM_HW_PARAM_PERIODS);
    if (ret < 0)
    {
        printk("5mic--->snd_pcm_hw_constraint_integer failed\n");
    }

    if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
    {

    }
    else
    {

    }

    if (ret < 0)
    {
        printk("5mic--->mtk_capture_pcm_close\n");
        mtk_capture_pcm_close(substream);
        return ret;
    }
    if(mCaptureUseSram == false)
    {
        AudDrv_Emi_Clk_On();
    }
    printk("5mic--->mtk_capture_pcm_open return\n");
    return 0;
}

static int mtk_capture_pcm_close(struct snd_pcm_substream *substream)
{
    if(mCaptureUseSram == false)
    {
        AudDrv_Emi_Clk_Off();
    }
    if (mCaptureUseSram == true)
    {
        ClearSramState(SRAM_STATE_CAPTURE);
        mCaptureUseSram = false;
    }
  AudDrv_Clk_Off();
  if(substream->runtime->rate == Samp_96)
  {
	   AudDrv_I2S_Clk_Off();  
	   AudDrv_ANA_Clk_Off();
  }
  else
   AudDrv_ADC_Clk_Off();
	 printk("5mic--->mtk_capture_pcm_close\n");
    return 0;
}

static int mtk_capture_alsa_start(struct snd_pcm_substream *substream)
{
    printk("5mic--->mtk_capture_alsa_start \n");
	if(substream->runtime->rate == Samp_96)	
	    SetMemifSubStream(Soc_Aud_Digital_Block_MEM_AWB, substream);
	else
	    SetMemifSubStream(Soc_Aud_Digital_Block_MEM_VUL, substream);	
    StartAudioCaptureHardware(substream);
#ifdef DENALI_FPGA_EARLYPORTING //ccc early porting, copy from TurnOnDacPower() and ADC_LOOP_DAC_Func()
//    Afe_Set_Reg(AFE_SGEN_CON0, 0x24862862, 0xffffffff);

//    Ana_Set_Reg(PMIC_AFE_TOP_CON0, 0x0002, 0x0002);   //UL from sinetable
//    Ana_Set_Reg(PMIC_AFE_TOP_CON0, 0x0001, 0x0001);   //DL from sinetable
    
//    Ana_Set_Reg(AFE_SGEN_CFG0 , 0x0080 , 0xffff);
//    Ana_Set_Reg(AFE_SGEN_CFG1 , 0x0101 , 0xffff);

	Ana_Get_Reg(AFE_AUDIO_TOP_CON0);   //power on clock

	Ana_Get_Reg(AFUNC_AUD_CON2);
	Ana_Get_Reg(AFUNC_AUD_CON0); //sdm audio fifo clock power on
	Ana_Get_Reg(AFUNC_AUD_CON2); //sdm power on
	Ana_Get_Reg(AFUNC_AUD_CON2); //sdm fifo enable
	Ana_Get_Reg(AFE_DL_SDM_CON1); //set attenuation gain
	Ana_Get_Reg(AFE_UL_DL_CON0); //[0] afe enable
	
    Ana_Get_Reg(AFE_PMIC_NEWIF_CFG0); //8k sample rate
    Ana_Get_Reg(AFE_DL_SRC2_CON0_H);//8k sample rate
    Ana_Get_Reg(AFE_DL_SRC2_CON0_L); //turn off mute function and turn on dl
    Ana_Get_Reg(PMIC_AFE_TOP_CON0); //set DL in normal path, not from sine gen table
    Ana_Get_Reg(AFE_SGEN_CFG0); //set DL in normal path, not from sine gen table
    Ana_Get_Reg(AFE_SGEN_CFG1); //set DL in normal path, not from sine gen table
    
    Ana_Get_Reg(TOP_CLKSQ); //Enable CLKSQ 26MHz
    Ana_Get_Reg(TOP_CLKSQ_SET); //Turn on 26MHz source clock        
    Ana_Get_Reg(AFE_AUDIO_TOP_CON0);   //power on clock
    
    Ana_Get_Reg(FPGA_CFG1); // must set in FPGA platform for PMIC digital loopback
#endif
    return 0;
}
extern bool mic_run_flag;

static int mtk_capture_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
  //  printk("5mic--->mtk_capture_pcm_trigger cmd = %d,,%s\n", cmd,__TIME__);
	int ret;
    switch (cmd)
    {
        case SNDRV_PCM_TRIGGER_START:
    //    case SNDRV_PCM_TRIGGER_RESUME:
		if(substream->stream ==SNDRV_PCM_STREAM_CAPTURE)
		{
			if(substream->runtime->rate == Samp_96)
			{
				fpga_set_gpio_output(GPIO_RST_EN,1);
			}
			
		}
		mic_run_flag=true;
            return mtk_capture_alsa_start(substream);
        case SNDRV_PCM_TRIGGER_STOP:
      //  case SNDRV_PCM_TRIGGER_SUSPEND:		
		ret=mtk_capture_alsa_stop(substream);
		if(substream->stream ==SNDRV_PCM_STREAM_CAPTURE)
		{
			if(substream->runtime->rate == Samp_96)
			{
			fpga_set_gpio_output(GPIO_RST_EN,0);
			}
		}
		mic_run_flag=false;
            return ret;
    }
    return -EINVAL;
}

static bool CheckNullPointer(void *pointer)
{
    if (pointer == NULL)
    {
        printk("5mic--->CheckNullPointer pointer = NULL");
        return true;
    }
    return false;
}

static int mtk_capture_pcm_copy(struct snd_pcm_substream *substream,
                                int channel, snd_pcm_uframes_t pos,
                                void __user *dst, snd_pcm_uframes_t count)
{

    AFE_MEM_CONTROL_T *pVUL_MEM_ConTrol = NULL;
    AFE_BLOCK_T  *Vul_Block = NULL;
    char *Read_Data_Ptr = (char *)dst;
//	static int i=0 ;
    ssize_t DMA_Read_Ptr = 0 , read_size = 0, read_count = 0;
    //struct snd_pcm_runtime *runtime = substream->runtime;
    unsigned long flags;
	

   // PRINTK_AUD_UL1("5mic--->mtk_capture_pcm_copy pos = %lucount = %lu     time=%s\n ", pos, count,__TIME__);
    // get total bytes to copy
    count = Align64ByteSize(audio_frame_to_bytes(substream , count));

    // check which memif nned to be write
    pVUL_MEM_ConTrol = VUL_Control_context;
    Vul_Block = &(pVUL_MEM_ConTrol->rBlock);

    if (pVUL_MEM_ConTrol == NULL)
    {
        printk("5mic--->cannot find MEM control !!!!!!!\n");
        msleep(50);
        return 0;
    }

    if (Vul_Block->u4BufferSize <= 0)
    {
        msleep(50);
        printk("5mic--->Vul_Block->u4BufferSize <= 0  =%d\n", Vul_Block->u4BufferSize);
        return 0;
    }

    if (CheckNullPointer((void *)Vul_Block->pucVirtBufAddr))
    {
        printk("5mic--->CheckNullPointer  pucVirtBufAddr = %p\n", Vul_Block->pucVirtBufAddr);
        return 0;
    }

    spin_lock_irqsave(&auddrv_ULInCtl_lock, flags);
    if (Vul_Block->u4DataRemained >  Vul_Block->u4BufferSize)
    {
        PRINTK_AUD_UL1("5mic--->AudDrv_MEMIF_Read u4DataRemained=%x > u4BufferSize=%x" , Vul_Block->u4DataRemained, Vul_Block->u4BufferSize);
        Vul_Block->u4DataRemained = 0;
        Vul_Block->u4DMAReadIdx   = Vul_Block->u4WriteIdx;
    }
    if (count >  Vul_Block->u4DataRemained)
    {
        read_size = Vul_Block->u4DataRemained;
    }
    else
    {
        read_size = count;
    }

    DMA_Read_Ptr = Vul_Block->u4DMAReadIdx;
    spin_unlock_irqrestore(&auddrv_ULInCtl_lock, flags);

    PRINTK_AUD_UL1("5mic--->AudDrv_MEMIF_Read finish0, read_count:%x, read_size:%x, u4DataRemained:%x, u4DMAReadIdx:0x%x, u4WriteIdx:%x \r\n",
                   (unsigned int)read_count, (unsigned int)read_size, Vul_Block->u4DataRemained, Vul_Block->u4DMAReadIdx, Vul_Block->u4WriteIdx);

    if (DMA_Read_Ptr + read_size < Vul_Block->u4BufferSize)
    {
        if (DMA_Read_Ptr != Vul_Block->u4DMAReadIdx)
        {
            printk("5mic--->AudDrv_MEMIF_Read 1, read_size:%zu, DataRemained:%x, DMA_Read_Ptr:0x%zu, DMAReadIdx:%x \r\n",
                   read_size, Vul_Block->u4DataRemained, DMA_Read_Ptr, Vul_Block->u4DMAReadIdx);
        }
      if (copy_to_user((void __user *)Read_Data_Ptr, (Vul_Block->pucVirtBufAddr + DMA_Read_Ptr), read_size))     
        {

            printk("5mic--->AudDrv_MEMIF_Read Fail 1 copy to user Read_Data_Ptr:%p, pucVirtBufAddr:%p, u4DMAReadIdx:0x%x, DMA_Read_Ptr:%zu,read_size:%zu", Read_Data_Ptr, Vul_Block->pucVirtBufAddr, Vul_Block->u4DMAReadIdx, DMA_Read_Ptr, read_size);
            return 0;
        }
        read_count += read_size;
        spin_lock(&auddrv_ULInCtl_lock);
        Vul_Block->u4DataRemained -= read_size;
        Vul_Block->u4DMAReadIdx += read_size;
        Vul_Block->u4DMAReadIdx %= Vul_Block->u4BufferSize;
        DMA_Read_Ptr = Vul_Block->u4DMAReadIdx;
        spin_unlock(&auddrv_ULInCtl_lock);

        Read_Data_Ptr += read_size;
        count -= read_size;

        PRINTK_AUD_UL1("5mic--->AudDrv_MEMIF_Read finish1, copy size:%x, u4DMAReadIdx:0x%x, u4WriteIdx:%x, u4DataRemained:%x \r\n",
                       (unsigned int)read_size, Vul_Block->u4DMAReadIdx, Vul_Block->u4WriteIdx, Vul_Block->u4DataRemained);
    }

    else
    {
        uint32 size_1 = Vul_Block->u4BufferSize - DMA_Read_Ptr;
        uint32 size_2 = read_size - size_1;
       PRINTK_AUD_UL1("5mic--->AudDrv_MEMIF_Read  ---else---\r\n");

        if (DMA_Read_Ptr != Vul_Block->u4DMAReadIdx)
        {

            printk("5mic--->AudDrv_MEMIF_Read 2, read_size1:%x, DataRemained:%x, DMA_Read_Ptr:%zu, DMAReadIdx:%x \r\n",
                   size_1, Vul_Block->u4DataRemained, DMA_Read_Ptr, Vul_Block->u4DMAReadIdx);
        }
        if (copy_to_user((void __user *)Read_Data_Ptr, (Vul_Block->pucVirtBufAddr + DMA_Read_Ptr), (unsigned int)size_1))
        {

            printk("5mic--->AudDrv_MEMIF_Read Fail 2 copy to user Read_Data_Ptr:%p, pucVirtBufAddr:%p, u4DMAReadIdx:0x%x, DMA_Read_Ptr:%zu,read_size:%zu",
                   Read_Data_Ptr, Vul_Block->pucVirtBufAddr, Vul_Block->u4DMAReadIdx, DMA_Read_Ptr, read_size);
            return 0;
        }

        read_count += size_1;
        spin_lock(&auddrv_ULInCtl_lock);
        Vul_Block->u4DataRemained -= size_1;
        Vul_Block->u4DMAReadIdx += size_1;
        Vul_Block->u4DMAReadIdx %= Vul_Block->u4BufferSize;
        DMA_Read_Ptr = Vul_Block->u4DMAReadIdx;
        spin_unlock(&auddrv_ULInCtl_lock);


        PRINTK_AUD_UL1("5mic--->AudDrv_MEMIF_Read finish2, copy size_1:%x, u4DMAReadIdx:0x%x, u4WriteIdx:0x%x, u4DataRemained:%x \r\n",
                       size_1, Vul_Block->u4DMAReadIdx, Vul_Block->u4WriteIdx, Vul_Block->u4DataRemained);

        if (DMA_Read_Ptr != Vul_Block->u4DMAReadIdx)
        {

            printk("5mic--->AudDrv_AWB_Read 3, read_size2:%x, DataRemained:%x, DMA_Read_Ptr:%zu, DMAReadIdx:%x \r\n",
                   size_2, Vul_Block->u4DataRemained, DMA_Read_Ptr, Vul_Block->u4DMAReadIdx);
        }
        if (copy_to_user((void __user *)(Read_Data_Ptr + size_1), (Vul_Block->pucVirtBufAddr + DMA_Read_Ptr), size_2))
        {

            printk("5mic--->AudDrv_MEMIF_Read Fail 3 copy to user Read_Data_Ptr:%p, pucVirtBufAddr:%p, u4DMAReadIdx:0x%x , DMA_Read_Ptr:%zu, read_size:%zu", Read_Data_Ptr, Vul_Block->pucVirtBufAddr, Vul_Block->u4DMAReadIdx, DMA_Read_Ptr, read_size);
            return read_count << 2;
        }
        read_count += size_2;
        spin_lock(&auddrv_ULInCtl_lock);
        Vul_Block->u4DataRemained -= size_2;
        Vul_Block->u4DMAReadIdx += size_2;
        DMA_Read_Ptr = Vul_Block->u4DMAReadIdx;
        spin_unlock(&auddrv_ULInCtl_lock);

        count -= read_size;
        Read_Data_Ptr += read_size;

        PRINTK_AUD_UL1("5mic---> AudDrv_MEMIF_Read finish3, copy size_2:%x, u4DMAReadIdx:0x%x, u4WriteIdx:0x%x u4DataRemained:%x \r\n",
                       size_2, Vul_Block->u4DMAReadIdx, Vul_Block->u4WriteIdx, Vul_Block->u4DataRemained);
    }

	PRINTK_AUD_UL1("5mic--->AudDrv_MEMIF_Read  ---finish\r\n");
						
    return read_count >> 2;
}

static int mtk_capture_pcm_silence(struct snd_pcm_substream *substream,
                                   int channel, snd_pcm_uframes_t pos,
                                   snd_pcm_uframes_t count)
{
    printk("5mic--->dummy_pcm_silence \n");
    return 0; /* do nothing */
}


static void *dummy_page[2];

static struct page *mtk_capture_pcm_page(struct snd_pcm_substream *substream,
                                         unsigned long offset)
{
    printk("%s 5mic--->\n", __func__);
    return virt_to_page(dummy_page[substream->stream]); /* the same page */
}


static struct snd_pcm_ops mtk_afe_capture_ops =
{
    .open =     mtk_capture_pcm_open,
    .close =    mtk_capture_pcm_close,
    .ioctl =    snd_pcm_lib_ioctl,
    .hw_params =    mtk_capture_pcm_hw_params,
    .hw_free =  mtk_capture_pcm_hw_free,
    .prepare =  mtk_capture_pcm_prepare,
    .trigger =  mtk_capture_pcm_trigger,
    .pointer =  mtk_capture_pcm_pointer,
    .copy =     mtk_capture_pcm_copy,
    .silence =  mtk_capture_pcm_silence,
    .page =     mtk_capture_pcm_page,
};

static struct snd_soc_platform_driver mtk_soc_platform =
{
    .ops        = &mtk_afe_capture_ops,
    .pcm_new    = mtk_asoc_capture_pcm_new,
    .probe      = mtk_afe_capture_probe,
};

static int mtk_capture_probe(struct platform_device *pdev)
{
    printk("5mic--->mtk_capture_probe\n");

    pdev->dev.coherent_dma_mask = DMA_BIT_MASK(64);
    if (pdev->dev.dma_mask == NULL)
    {
        pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
    }

    if (pdev->dev.of_node)
    {
        dev_set_name(&pdev->dev, "%s", MT_SOC_5MIC_PCM);
    }

    printk("%s: dev name %s\n", __func__, dev_name(&pdev->dev));
    return snd_soc_register_platform(&pdev->dev,
                                     &mtk_soc_platform);
}

static int mtk_asoc_capture_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
    printk(" 5mic---> mtk_asoc_capture_pcm_new \n");
    return 0;
}


static int mtk_afe_capture_probe(struct snd_soc_platform *platform)
{
    printk("mtk_afe_capture_probe  5mic\n");//  
    //@mt_soc_pcm_capture.c   allocate dma
    
  //  AudDrv_Allocate_mem_Buffer(platform->dev, Soc_Aud_Digital_Block_MEM_AWB, AWB_MAX_BUFFER_SIZE);
 //   AWBCapture_dma_buf =  Get_Mem_Buffer(Soc_Aud_Digital_Block_MEM_AWB);

      mAudioDigitalI2S =  kzalloc(sizeof(AudioDigtalI2S), GFP_KERNEL);
    return 0;
}


static int mtk_capture_remove(struct platform_device *pdev)
{
    pr_debug("%s\n", __func__);
    snd_soc_unregister_platform(&pdev->dev);
    return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id mt_soc_pcm_capture_of_ids[] =
{
    { .compatible = "mediatek,mt_soc_pcm_capture_5mic", },
    {}
};
#endif

static struct platform_driver mtk_afe_capture_driver =
{
    .driver = {
        .name = MT_SOC_5MIC_PCM,
        .owner = THIS_MODULE,
        #ifdef CONFIG_OF
        .of_match_table = mt_soc_pcm_capture_of_ids,
        #endif        
    },
    .probe = mtk_capture_probe,
    .remove = mtk_capture_remove,
};

static void fpga_get_gpio_infor(void)
{
	static struct device_node *node;
	node = of_find_compatible_node(NULL, NULL, "mediatek,fpga");
	GPIO_RST_EN = of_get_named_gpio(node, "rst_d3_gpio124", 0);
	GPIO_89 = of_get_named_gpio(node, "misc_u5_gpio89", 0);
	GPIO_5 = of_get_named_gpio(node, "misc_l19_gpio5", 0);
	GPIO_42 = of_get_named_gpio(node, "misc_y18_gpio42", 0);
	
}

static void fpga_set_gpio_output(unsigned int GPIO, unsigned int output)
{
	gpio_direction_output(GPIO, output);
	gpio_set_value(GPIO, output);	
}

/* get LDO supply */
static int fpga_probe(struct device *dev)
{	
	printk("lifei++11++++fpga_probe\n");
	fpga_get_gpio_infor();
	gpio_request(GPIO_RST_EN, "GPIO_RST_EN");
	gpio_request(GPIO_42, "GPIO_42");
	gpio_request(GPIO_89, "GPIO_89");
	gpio_request(GPIO_5, "GPIO_5");
	
	fpga_set_gpio_output(GPIO_RST_EN,0);
	fpga_set_gpio_output(GPIO_89,1);
	fpga_set_gpio_output(GPIO_5,1);
	fpga_set_gpio_output(GPIO_42,0);

	wake_lock_init(&m_lock, WAKE_LOCK_SUSPEND, "misc"); //davie: forbid deep sleep for 5mic
	wake_lock(&m_lock);
	return 0;
}

static const struct of_device_id fpga_of_ids[] = {
	{.compatible = "mediatek,fpga",},
	{}
};

static struct platform_driver fpga_driver = {
	.driver = {
		   .name = "mtk_fpga",
		   .owner = THIS_MODULE,
		   .probe = fpga_probe,
#ifdef CONFIG_OF
		   .of_match_table = fpga_of_ids,
#endif
		   },
};

#ifndef CONFIG_OF
static struct platform_device *soc_mtkafe_capture_dev;
#endif

static int __init mtk_soc_capture_platform_init(void)
{
    int ret = 0;
    printk("%s  mic\n", __func__);
	#ifndef CONFIG_OF
    soc_mtkafe_capture_dev = platform_device_alloc(MT_SOC_5MIC_PCM, -1);
    if (!soc_mtkafe_capture_dev)
    {
        return -ENOMEM;
    }

    ret = platform_device_add(soc_mtkafe_capture_dev);
    if (ret != 0)
    {
        platform_device_put(soc_mtkafe_capture_dev);
        return ret;
    }
    #endif
    ret = platform_driver_register(&mtk_afe_capture_driver);
    ret = platform_driver_register(&fpga_driver);
    return ret;
}


static void __exit mtk_soc_platform_exit(void)
{

    printk("%s\n", __func__);
    platform_driver_unregister(&mtk_afe_capture_driver);
}
module_init(mtk_soc_capture_platform_init);
module_exit(mtk_soc_platform_exit);
MODULE_DESCRIPTION("AFE PCM module platform driver 5mic");
MODULE_LICENSE("GPL");


