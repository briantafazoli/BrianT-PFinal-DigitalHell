/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin processor.

  ==============================================================================
*/

#include "PluginProcessor.h"
#include "PluginEditor.h"

//==============================================================================
BrianTPFinalDigitalHellAudioProcessor::BrianTPFinalDigitalHellAudioProcessor()
#ifndef JucePlugin_PreferredChannelConfigurations
     : AudioProcessor (BusesProperties()
                     #if ! JucePlugin_IsMidiEffect
                      #if ! JucePlugin_IsSynth
                       .withInput  ("Input",  juce::AudioChannelSet::stereo(), true)
                      #endif
                       .withOutput ("Output", juce::AudioChannelSet::stereo(), true)
                     #endif
                       )
#endif
{

    // ====================================================== FILTERS =================================================================

    // Low Band Hi Pass Filter CutOff Frequency
    addParameter(mLowHPFCutoffFreqParam = new AudioParameterFloat("LowHPFCutoffFreq", // parameterID,
        "Low Band HPF Cutoff Frequency (Hz)", // parameterName,
        20.0f, // minValue,
        500.0f, // maxValue,
        50.0f)); // defaultValue

    // Low Band Low Pass / Mid Band High Pass Filter Cutoff Frequency
    addParameter(mLowLPFCutoffFreqParam = new AudioParameterFloat("LowLPFCutoffFreq", // parameterID,
        "Low Band LPF / Mid Band HPF Cutoff Frequency (Hz)", // parameterName,
        50.0f, // minValue,
        5000.0f, // maxValue,
        500.0f)); // defaultValue

    // High Band High Pass / Mid Band Low Pass Filter Cutoff Frequency
    addParameter(mHighHPFCutoffFreqParam = new AudioParameterFloat("HighHPFCutoffFreq", // parameterID,
        "High Band HPF / Mid Band LPF Cutoff Frequency (Hz)", // parameterName,
        500.0f, // minValue,
        15000.0f, // maxValue,
        5000.0f)); // defaultValue

    // High Band Low Pass Filter Cutoff Frequency
    addParameter(mHighLPFCutoffFreqParam = new juce::AudioParameterFloat("HighLPFCutoffFreq", // parameterID,
        "High Band LPF Cutoff Frequency (Hz)", // parameterName,
        2500.0f, // minValue,
        20000.0f, // maxValue,
        15000.0f)); // defaultValue

    // ================================================ BITCRUSH PARAMS ===============================================================

    // Low Band Sample Rate
    addParameter(mLowDWSPParam = new juce::AudioParameterFloat("lowDWSP", // parameterID,
        "Low Band Downsample %", // parameterName,
        0.0f, // minValue,
        100.0f, // maxValue,
        20.0f)); // defaultValue

    // Mid Band Sample Rate
    addParameter(mMidDWSPParam = new juce::AudioParameterFloat("MidDWSP", // parameterID,
        "Mid Band Downsample %", // parameterName,
        0.0f, // minValue,
        100.0f, // maxValue,
        20.0f)); // defaultValue

    // High Band Sample Rate
    addParameter(mHighDWSPParam = new juce::AudioParameterFloat("highDWSP", // parameterID,
        "High Band Downsample %", // parameterName,
        0.0f, // minValue,
        100.0f, // maxValue,
        20.0f)); // defaultValue

    // ============================================== GAIN ===================================================================

    // Low Band Gain
    addParameter(mDBLowGainParam = new juce::AudioParameterFloat("LoGain", // parameterID,
        "Low Band Gain", // parameterName,
        -60.0f, // minValue,
        6.0f, // maxValue,
        0.0f)); // defaultValue

    // Mid Band Gain
    addParameter(mDBMidGainParam = new juce::AudioParameterFloat("MidGain", // parameterID,
        "Mid Band Gain", // parameterName,
        -60.0f, // minValue,
        6.0f, // maxValue,
        0.0f)); // defaultValue

    // High Band Gain
    addParameter(mDBHighGainParam = new juce::AudioParameterFloat("HiGain", // parameterID,
        "High Band Gain", // parameterName,
        -60.0f, // minValue,
        6.0f, // maxValue,
        0.0f)); // defaultValue

    // ========================================================== ENABLE ============================================================

    addParameter(mLowEnabledBool = new AudioParameterBool("LoEnable", //ID
        "Low Band Enabled", // name
        true)); // default

    addParameter(mMidEnabledBool = new AudioParameterBool("MidEnable", //ID
        "Mid Band Enabled", // name
        true)); // default

    addParameter(mHighEnabledBool = new AudioParameterBool("HiEnable", //ID
        "High Band Enabled", // name
        true)); // default

}

BrianTPFinalDigitalHellAudioProcessor::~BrianTPFinalDigitalHellAudioProcessor()
{
}

//==============================================================================
const juce::String BrianTPFinalDigitalHellAudioProcessor::getName() const
{
    return JucePlugin_Name;
}

bool BrianTPFinalDigitalHellAudioProcessor::acceptsMidi() const
{
   #if JucePlugin_WantsMidiInput
    return true;
   #else
    return false;
   #endif
}

bool BrianTPFinalDigitalHellAudioProcessor::producesMidi() const
{
   #if JucePlugin_ProducesMidiOutput
    return true;
   #else
    return false;
   #endif
}

bool BrianTPFinalDigitalHellAudioProcessor::isMidiEffect() const
{
   #if JucePlugin_IsMidiEffect
    return true;
   #else
    return false;
   #endif
}

double BrianTPFinalDigitalHellAudioProcessor::getTailLengthSeconds() const
{
    return 0.0;
}

int BrianTPFinalDigitalHellAudioProcessor::getNumPrograms()
{
    return 1;   // NB: some hosts don't cope very well if you tell them there are 0 programs,
                // so this should be at least 1, even if you're not really implementing programs.
}

int BrianTPFinalDigitalHellAudioProcessor::getCurrentProgram()
{
    return 0;
}

void BrianTPFinalDigitalHellAudioProcessor::setCurrentProgram (int index)
{
}

const juce::String BrianTPFinalDigitalHellAudioProcessor::getProgramName (int index)
{
    return {};
}

void BrianTPFinalDigitalHellAudioProcessor::changeProgramName (int index, const juce::String& newName)
{
}

//==============================================================================
void BrianTPFinalDigitalHellAudioProcessor::prepareToPlay (double sampleRate, int samplesPerBlock)
{
    // Use this method as the place to do any pre-playback
    // initialisation that you need..

    mFs = sampleRate;

}

void BrianTPFinalDigitalHellAudioProcessor::releaseResources()
{
    // When playback stops, you can use this as an opportunity to free up any
    // spare memory, etc.
}

#ifndef JucePlugin_PreferredChannelConfigurations
bool BrianTPFinalDigitalHellAudioProcessor::isBusesLayoutSupported (const BusesLayout& layouts) const
{
  #if JucePlugin_IsMidiEffect
    juce::ignoreUnused (layouts);
    return true;
  #else
    // This is the place where you check if the layout is supported.
    // In this template code we only support mono or stereo.
    // Some plugin hosts, such as certain GarageBand versions, will only
    // load plugins that support stereo bus layouts.
    if (layouts.getMainOutputChannelSet() != juce::AudioChannelSet::mono()
     && layouts.getMainOutputChannelSet() != juce::AudioChannelSet::stereo())
        return false;

    // This checks if the input layout matches the output layout
   #if ! JucePlugin_IsSynth
    if (layouts.getMainOutputChannelSet() != layouts.getMainInputChannelSet())
        return false;
   #endif

    return true;
  #endif
}
#endif

void BrianTPFinalDigitalHellAudioProcessor::processBlock(juce::AudioBuffer<float>& buffer, juce::MidiBuffer& midiMessages)
{
    // JUCE book-keeping to make sure there's no bad data in the output buffers
    juce::ScopedNoDenormals noDenormals;
    auto totalNumInputChannels = getTotalNumInputChannels();
    auto totalNumOutputChannels = getTotalNumOutputChannels();
    for (auto i = totalNumInputChannels; i < totalNumOutputChannels; ++i)
        buffer.clear(i, 0, buffer.getNumSamples());
    // Update the algorithm params
    calcAlgorithmParams();
    // Get the left and right audio buffers
    auto* channelDataLeft = buffer.getWritePointer(0);
    auto* channelDataRight = buffer.getWritePointer(1);

    int numSamples = buffer.getNumSamples();

    float* lowChannelDataL = new float[numSamples];
    float* lowChannelDataR = new float[numSamples];
    float* midChannelDataL = new float[numSamples];
    float* midChannelDataR = new float[numSamples];
    float* highChannelDataL = new float[numSamples];
    float* highChannelDataR = new float[numSamples];

    float lowCurrentSampL;
    float lowCurrentSampR;
    float midCurrentSampL;
    float midCurrentSampR;
    float highCurrentSampL;
    float highCurrentSampR;

    int lowSampleCounter = 0;
    int midSampleCounter = 0;
    int highSampleCounter = 0;

    // The inner loop! Step through each sample in the audio buffer
    for (int samp = 0; samp < buffer.getNumSamples(); samp++) {

        if (lowEnabled) {

            if (lowDWSP == 0) {
                // No downsampling: process every sample directly
                // Without this block of code, it will still downsample slightly.
                lowChannelDataL[samp] = tickSampleData(channelDataLeft[samp], &mLowHPFilter, &mLowLPFilter) * linearGainLow;
                lowChannelDataR[samp] = tickSampleData(channelDataRight[samp], &mLowHPFilter, &mLowLPFilter) * linearGainLow;
            }

            else {

                if (lowSampleCounter == 0) {

                    lowCurrentSampL = channelDataLeft[samp];
                    lowCurrentSampR = channelDataRight[samp];

                }

                lowChannelDataL[samp] = tickSampleData(lowCurrentSampL, &mLowHPFilter, &mLowLPFilter) * linearGainLow;
                lowChannelDataR[samp] = tickSampleData(lowCurrentSampR, &mLowHPFilter, &mLowLPFilter) * linearGainLow;

                if (lowSampleCounter == lowDWSP) lowSampleCounter = -1;

                lowSampleCounter++;

            }     

        }

        else {

            lowChannelDataL[samp] = 0;
            lowChannelDataR[samp] = 0;

        }

        if (midEnabled) {

            if (midDWSP == 0) {
                // No downsampling: process every sample directly
                // Without this block of code, it will still downsample slightly.
                midChannelDataL[samp] = tickSampleData(channelDataLeft[samp], &mMidHPFilter, &mMidLPFilter) * linearGainMid;
                midChannelDataR[samp] = tickSampleData(channelDataRight[samp], &mMidHPFilter, &mMidLPFilter) * linearGainMid;
            }

            else {

                if (midSampleCounter == 0) {

                    midCurrentSampL = channelDataLeft[samp];
                    midCurrentSampR = channelDataRight[samp];

                }

                midChannelDataL[samp] = tickSampleData(midCurrentSampL, &mMidHPFilter, &mMidLPFilter) * linearGainMid;
                midChannelDataR[samp] = tickSampleData(midCurrentSampR, &mMidHPFilter, &mMidLPFilter) * linearGainMid;

                if (midSampleCounter == midDWSP) midSampleCounter = -1;

                midSampleCounter++;

            }

        }

        else {

            midChannelDataL[samp] = 0;
            midChannelDataR[samp] = 0;

        }


        if (highEnabled) {

            if (highDWSP == 0) {
                // No downsampling: process every sample directly
                // Without this block of code, it will still downsample slightly.
                highChannelDataL[samp] = tickSampleData(channelDataLeft[samp], &mHighHPFilter, &mHighLPFilter) * linearGainHigh;
                highChannelDataR[samp] = tickSampleData(channelDataRight[samp], &mHighHPFilter, &mHighLPFilter) * linearGainHigh;
            }

            else {

                if (highSampleCounter == 0) {

                    highCurrentSampL = channelDataLeft[samp];
                    highCurrentSampR = channelDataRight[samp];

                }

                highChannelDataL[samp] = tickSampleData(highCurrentSampL, &mHighHPFilter, &mHighLPFilter) * linearGainHigh;
                highChannelDataR[samp] = tickSampleData(highCurrentSampR, &mHighHPFilter, &mHighLPFilter) * linearGainHigh;

                if (highSampleCounter == highDWSP) highSampleCounter = -1;

                highSampleCounter++;

            }

        }

        else {

            highChannelDataL[samp] = 0;
            highChannelDataR[samp] = 0;

        }

        /*Since the bitcrushing can still introduce artifacts, we need to sample them once again.*/
        

        channelDataLeft[samp] = lowChannelDataL[samp] + midChannelDataL[samp] + highChannelDataL[samp];
        channelDataRight[samp] = lowChannelDataR[samp] + midChannelDataR[samp] + highChannelDataR[samp];

        }

        /*delete[] lowChannelDataL;
        delete[] lowChannelDataR;
        delete[] midChannelDataL;
        delete[] midChannelDataR;
        delete[] highChannelDataL;
        delete[] highChannelDataR;*/

}




void BrianTPFinalDigitalHellAudioProcessor::calcAlgorithmParams() {

    float loHPFc = mLowHPFCutoffFreqParam->get();
    float loLPFc = mLowLPFCutoffFreqParam->get();
    float midHPFc = mLowLPFCutoffFreqParam->get();
    float midLPFc = mHighHPFCutoffFreqParam->get();
    float hiHPFc = mHighHPFCutoffFreqParam->get();
    float hiLPFc = mHighLPFCutoffFreqParam->get();

    linearGainLow = pow(10, (mDBLowGainParam->get() / 20));
    linearGainMid = pow(10, (mDBMidGainParam->get() / 20));
    linearGainHigh = pow(10, (mDBHighGainParam->get() / 20));

    float Q = 0.707f;

    float loHPFCoeffs[5];
    Mu45FilterCalc::calcCoeffsHPF(loHPFCoeffs, loHPFc, Q, mFs);

    float loLPFCoeffs[5];
    Mu45FilterCalc::calcCoeffsLPF(loLPFCoeffs, loLPFc, Q, mFs);

    float midHPFCoeffs[5];
    Mu45FilterCalc::calcCoeffsHPF(midHPFCoeffs, midHPFc, Q, mFs);

    float midLPFCoeffs[5];
    Mu45FilterCalc::calcCoeffsLPF(midLPFCoeffs, midLPFc, Q, mFs);

    float hiHPFCoeffs[5];
    Mu45FilterCalc::calcCoeffsHPF(hiHPFCoeffs, hiHPFc, Q, mFs);

    float hiLPFCoeffs[5];
    Mu45FilterCalc::calcCoeffsLPF(hiLPFCoeffs, hiLPFc, Q, mFs);

    mLowHPFilter.setCoefficients(loHPFCoeffs[0], loHPFCoeffs[1], loHPFCoeffs[2], loHPFCoeffs[3], loHPFCoeffs[4]);
    mLowLPFilter.setCoefficients(loLPFCoeffs[0], loLPFCoeffs[1], loLPFCoeffs[2], loLPFCoeffs[3], loLPFCoeffs[4]);

    mMidHPFilter.setCoefficients(midHPFCoeffs[0], midHPFCoeffs[1], midHPFCoeffs[2], midHPFCoeffs[3], midHPFCoeffs[4]);
    mMidLPFilter.setCoefficients(midLPFCoeffs[0], midLPFCoeffs[1], midLPFCoeffs[2], midLPFCoeffs[3], midLPFCoeffs[4]);

    mHighHPFilter.setCoefficients(hiHPFCoeffs[0], hiHPFCoeffs[1], hiHPFCoeffs[2], hiHPFCoeffs[3], hiHPFCoeffs[4]);
    mHighLPFilter.setCoefficients(hiLPFCoeffs[0], hiLPFCoeffs[1], hiLPFCoeffs[2], hiLPFCoeffs[3], hiLPFCoeffs[4]);

    lowDWSP = static_cast<int> (mLowDWSPParam->get() + (maxDWSP - 1.0f) / 100.0f);
    midDWSP = static_cast<int> (mMidDWSPParam->get() + (maxDWSP - 1.0f) / 100.0f);
    highDWSP = static_cast<int> (mHighDWSPParam->get() + (maxDWSP - 1.0f) / 100.0f);

    lowEnabled = mLowEnabledBool->get();
    midEnabled = mMidEnabledBool->get();
    highEnabled = mHighEnabledBool->get();

}

void BrianTPFinalDigitalHellAudioProcessor::soloLowBand(bool isEnabled) { 

    if (isEnabled) {
        mLowEnabledBool->setValueNotifyingHost(true);
        mMidEnabledBool->setValueNotifyingHost(false);
        mHighEnabledBool->setValueNotifyingHost(false);
    }
    else {
        mLowEnabledBool->setValueNotifyingHost(true);
        mMidEnabledBool->setValueNotifyingHost(true);
        mHighEnabledBool->setValueNotifyingHost(true);
    }

}

void BrianTPFinalDigitalHellAudioProcessor::soloMidBand(bool isEnabled) {

    if (isEnabled) {
        mLowEnabledBool->setValueNotifyingHost(false);
        mMidEnabledBool->setValueNotifyingHost(true);
        mHighEnabledBool->setValueNotifyingHost(false);
    }
    else {
        mLowEnabledBool->setValueNotifyingHost(true);
        mMidEnabledBool->setValueNotifyingHost(true);
        mHighEnabledBool->setValueNotifyingHost(true);
    }

}

void BrianTPFinalDigitalHellAudioProcessor::soloHighBand(bool isEnabled) {

    if (isEnabled) {
        mLowEnabledBool->setValueNotifyingHost(false);
        mMidEnabledBool->setValueNotifyingHost(false);
        mHighEnabledBool->setValueNotifyingHost(true);
    }
    else {
        mLowEnabledBool->setValueNotifyingHost(true);
        mMidEnabledBool->setValueNotifyingHost(true);
        mHighEnabledBool->setValueNotifyingHost(true);
    }

}

float BrianTPFinalDigitalHellAudioProcessor::tickSampleData(float channelData, stk::BiQuad* HPFilter, stk::BiQuad* LPFilter) {

    float output = HPFilter->tick(channelData);
    output = LPFilter->tick(output);

    return output;

}

//==============================================================================
bool BrianTPFinalDigitalHellAudioProcessor::hasEditor() const
{
    return true; // (change this to false if you choose to not supply an editor)
}

juce::AudioProcessorEditor* BrianTPFinalDigitalHellAudioProcessor::createEditor()
{
    return new BrianTPFinalDigitalHellAudioProcessorEditor (*this);
}

//==============================================================================
void BrianTPFinalDigitalHellAudioProcessor::getStateInformation (juce::MemoryBlock& destData)
{
    // You should use this method to store your parameters in the memory block.
    // You could do that either as raw data, or use the XML or ValueTree classes
    // as intermediaries to make it easy to save and load complex data.
}

void BrianTPFinalDigitalHellAudioProcessor::setStateInformation (const void* data, int sizeInBytes)
{
    // You should use this method to restore your parameters from this memory block,
    // whose contents will have been created by the getStateInformation() call.
}

//==============================================================================
// This creates new instances of the plugin..
juce::AudioProcessor* JUCE_CALLTYPE createPluginFilter()
{
    return new BrianTPFinalDigitalHellAudioProcessor();
}
