/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin processor.

  ==============================================================================
*/

#pragma once

#include <JuceHeader.h>
#include <Mu45FilterCalc/Mu45FilterCalc/Mu45FilterCalc.h>
#include <StkLite-4.6.2/StkLite-4.6.2/BiQuad.h>

//==============================================================================
/**
*/
class BrianTPFinalDigitalHellAudioProcessor  : public juce::AudioProcessor
{
public:
    //==============================================================================
    BrianTPFinalDigitalHellAudioProcessor();
    ~BrianTPFinalDigitalHellAudioProcessor() override;

    //==============================================================================
    void prepareToPlay (double sampleRate, int samplesPerBlock) override;
    void releaseResources() override;

   #ifndef JucePlugin_PreferredChannelConfigurations
    bool isBusesLayoutSupported (const BusesLayout& layouts) const override;
   #endif

    void processBlock (juce::AudioBuffer<float>&, juce::MidiBuffer&) override;

    //==============================================================================
    juce::AudioProcessorEditor* createEditor() override;
    bool hasEditor() const override;

    //==============================================================================
    const juce::String getName() const override;

    bool acceptsMidi() const override;
    bool producesMidi() const override;
    bool isMidiEffect() const override;
    double getTailLengthSeconds() const override;

    //==============================================================================
    int getNumPrograms() override;
    int getCurrentProgram() override;
    void setCurrentProgram (int index) override;
    const juce::String getProgramName (int index) override;
    void changeProgramName (int index, const juce::String& newName) override;

    //==============================================================================
    void getStateInformation (juce::MemoryBlock& destData) override;
    void setStateInformation (const void* data, int sizeInBytes) override;

    //==============================================================================

    void calcAlgorithmParams();
    void soloLowBand(bool isEnabled);
    void soloMidBand(bool isEnabled);
    void soloHighBand(bool isEnabled);

    float tickSampleData(float channelData,  stk::BiQuad* HPFilter, stk::BiQuad* LPFilter);


private:
    //==============================================================================
    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR(BrianTPFinalDigitalHellAudioProcessor)

    stk::BiQuad mLowHPFilter;
    stk::BiQuad mLowLPFilter;
    stk::BiQuad mMidHPFilter;
    stk::BiQuad mMidLPFilter;
    stk::BiQuad mHighHPFilter;
    stk::BiQuad mHighLPFilter;

    AudioParameterFloat* mLowHPFCutoffFreqParam; // Dictates the highpass frequency for the low band
    AudioParameterFloat* mLowLPFCutoffFreqParam; // Dictates the lowpass frequency for the low band and the highpass frequency for the middle band
    AudioParameterFloat* mHighHPFCutoffFreqParam; // Dictates the highpass frequency for the high band and the lowpass frequency for the middle band
    AudioParameterFloat* mHighLPFCutoffFreqParam; // Dictates the lowpass frequency for the high band
    
    AudioParameterFloat* mLowDWSPParam; // Dictates the downsampling from the given sample rate for the low band
    AudioParameterFloat* mMidDWSPParam; // Dictates the downsampling from the given sample rate for the mid band
    AudioParameterFloat* mHighDWSPParam; // Dictates the downsampling from the given sample rate for the high band

    AudioParameterFloat* mDBLowGainParam;
    AudioParameterFloat* mDBMidGainParam;
    AudioParameterFloat* mDBHighGainParam;

    AudioParameterBool* mLowEnabledBool;
    AudioParameterBool* mMidEnabledBool;
    AudioParameterBool* mHighEnabledBool;

    bool lowEnabled = true;
    bool midEnabled = true;
    bool highEnabled = true;

    int lowDWSP; // Represents the amount of samples skipped by the processor for every tick on the low band
    int midDWSP; // Represents the amount of samples skipped by the processor for every tick on the mid band
    int highDWSP; // Represents the amount of samples skipped by the processor for every tick on the high band

    float linearGainLow;
    float linearGainMid;
    float linearGainHigh;

    static constexpr int maxDWSP = 128; // The max possible downsample rate;

    float mFs; // Inital Sample Rate

};
