/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin editor.

  ==============================================================================
*/

#pragma once

#include <JuceHeader.h>
#include "PluginProcessor.h"

//==============================================================================
/**
*/
class BrianTPFinalDigitalHellAudioProcessorEditor  : public AudioProcessorEditor, public Slider::Listener, public Button::Listener
{
public:
    BrianTPFinalDigitalHellAudioProcessorEditor (BrianTPFinalDigitalHellAudioProcessor&);
    ~BrianTPFinalDigitalHellAudioProcessorEditor() override;

    //==============================================================================
    void paint (juce::Graphics&) override;
    void resized() override;

    void sliderValueChanged(Slider* slider) override;
    void buttonClicked(Button* button) override;

private:
    // This reference is provided as a quick way for your editor to
    // access the processor object that created it.
    BrianTPFinalDigitalHellAudioProcessor& audioProcessor;

    Slider mLowHPFcSlider;
    Slider mLowLPMidHPFcSlider;
    Slider mMidLPHighHPFcSlider;
    Slider mHighLPFcSlider;

    Slider mLowDWSPSlider;
    Slider mMidDWSPSlider;
    Slider mHighDWSPSlider;

    Slider mLowDBSlider;
    Slider mMidDBSlider;
    Slider mHighDBSlider;

    ToggleButton mLowSoloEnable;
    ToggleButton mMidSoloEnable;
    ToggleButton mHighSoloEnable;


    juce::Rectangle<int> getSquare(float centerX, float centerY, float sideLength) {
        // Calculate and return the rectangle
        return juce::Rectangle<int>(
            centerX - (sideLength / 2), // Top-left X
            centerY - (sideLength / 2), // Top-left Y
            sideLength,                // Width
            sideLength                 // Height
        );
    };

    Image logo = ImageCache::getFromMemory(BinaryData::digitalHellLogo_png, BinaryData::digitalHellLogo_pngSize);

    JUCE_DECLARE_NON_COPYABLE_WITH_LEAK_DETECTOR (BrianTPFinalDigitalHellAudioProcessorEditor)
};
