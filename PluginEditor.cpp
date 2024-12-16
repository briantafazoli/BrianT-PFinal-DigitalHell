/*
  ==============================================================================

    This file contains the basic framework code for a JUCE plugin editor.

  ==============================================================================
*/

#include "PluginProcessor.h"
#include "PluginEditor.h"

//==============================================================================
BrianTPFinalDigitalHellAudioProcessorEditor::BrianTPFinalDigitalHellAudioProcessorEditor (BrianTPFinalDigitalHellAudioProcessor& p)
    : AudioProcessorEditor (&p), audioProcessor (p)
{
    // Make sure that before the constructor has finished, you've set the
    // editor's size to whatever you need it to be.
    setSize(875, 600);

    auto& params = processor.getParameters();
    AudioParameterFloat* audioParam;

    // LOW HPF FREQUENCY

    audioParam = (juce::AudioParameterFloat*)params.getUnchecked(0);

    mLowHPFcSlider.setSliderStyle(juce::Slider::SliderStyle::RotaryHorizontalVerticalDrag);
    mLowHPFcSlider.setTextBoxStyle(juce::Slider::TextBoxAbove, false, 50, 20);
    mLowHPFcSlider.setTextValueSuffix(" Hz");
    mLowHPFcSlider.setDoubleClickReturnValue(true, 50);
    mLowHPFcSlider.setNumDecimalPlacesToDisplay(0);
    mLowHPFcSlider.setRange(audioParam->range.start, audioParam->range.end);
    mLowHPFcSlider.setSkewFactorFromMidPoint(80);
    mLowHPFcSlider.setValue(*audioParam);
    mLowHPFcSlider.addListener(this);
    addAndMakeVisible(mLowHPFcSlider);

    // LOW LPF, MID HPF FREQUENCY

    audioParam = (juce::AudioParameterFloat*)params.getUnchecked(1);

    mLowLPMidHPFcSlider.setSliderStyle(juce::Slider::SliderStyle::RotaryHorizontalVerticalDrag);
    mLowLPMidHPFcSlider.setTextBoxStyle(juce::Slider::TextBoxAbove, false, 60, 20);
    mLowLPMidHPFcSlider.setTextValueSuffix(" Hz");
    mLowLPMidHPFcSlider.setDoubleClickReturnValue(true, 500);
    mLowLPMidHPFcSlider.setNumDecimalPlacesToDisplay(0);
    mLowLPMidHPFcSlider.setRange(audioParam->range.start, audioParam->range.end);
    mLowLPMidHPFcSlider.setSkewFactorFromMidPoint(500);
    mLowLPMidHPFcSlider.setValue(*audioParam);
    mLowLPMidHPFcSlider.addListener(this);
    addAndMakeVisible(mLowLPMidHPFcSlider);

    // MID LPF, HIGH HPF FREQUENCY

    audioParam = (juce::AudioParameterFloat*)params.getUnchecked(2);

    mMidLPHighHPFcSlider.setSliderStyle(juce::Slider::SliderStyle::RotaryHorizontalVerticalDrag);
    mMidLPHighHPFcSlider.setTextBoxStyle(juce::Slider::TextBoxAbove, false, 60, 20);
    mMidLPHighHPFcSlider.setTextValueSuffix(" Hz");
    mMidLPHighHPFcSlider.setDoubleClickReturnValue(true, 5000);
    mMidLPHighHPFcSlider.setNumDecimalPlacesToDisplay(0);
    mMidLPHighHPFcSlider.setRange(audioParam->range.start, audioParam->range.end);
    mMidLPHighHPFcSlider.setSkewFactorFromMidPoint(5000);
    mMidLPHighHPFcSlider.setValue(*audioParam);
    mMidLPHighHPFcSlider.addListener(this);
    addAndMakeVisible(mMidLPHighHPFcSlider);

    // HIGH LPF FREQUENCY

    audioParam = (juce::AudioParameterFloat*)params.getUnchecked(3);

    mHighLPFcSlider.setSliderStyle(juce::Slider::SliderStyle::RotaryHorizontalVerticalDrag);
    mHighLPFcSlider.setTextBoxStyle(juce::Slider::TextBoxAbove, false, 70, 20);
    mHighLPFcSlider.setTextValueSuffix(" Hz");
    mHighLPFcSlider.setDoubleClickReturnValue(true, 12000);
    mHighLPFcSlider.setNumDecimalPlacesToDisplay(0);
    mHighLPFcSlider.setRange(audioParam->range.start, audioParam->range.end);
    mHighLPFcSlider.setSkewFactorFromMidPoint(12000);
    mHighLPFcSlider.setValue(*audioParam);
    mHighLPFcSlider.addListener(this);
    addAndMakeVisible(mHighLPFcSlider);

    // LOW DWSP

    audioParam = (juce::AudioParameterFloat*)params.getUnchecked(4);

    mLowDWSPSlider.setSliderStyle(juce::Slider::SliderStyle::RotaryHorizontalVerticalDrag);
    mLowDWSPSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 40, 25);
    mLowDWSPSlider.setTextValueSuffix(" %");
    mLowDWSPSlider.setDoubleClickReturnValue(true, 20);
    mLowDWSPSlider.setNumDecimalPlacesToDisplay(0);
    mLowDWSPSlider.setRange(audioParam->range.start, audioParam->range.end);
    mLowDWSPSlider.setValue(*audioParam);
    mLowDWSPSlider.addListener(this);
    addAndMakeVisible(mLowDWSPSlider);


    // MID DWSP

    audioParam = (juce::AudioParameterFloat*)params.getUnchecked(5);

    mMidDWSPSlider.setSliderStyle(juce::Slider::SliderStyle::RotaryHorizontalVerticalDrag);
    mMidDWSPSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 40, 25);
    mMidDWSPSlider.setTextValueSuffix(" %");
    mMidDWSPSlider.setDoubleClickReturnValue(true, 20);
    mMidDWSPSlider.setNumDecimalPlacesToDisplay(0);
    mMidDWSPSlider.setRange(audioParam->range.start, audioParam->range.end);
    mMidDWSPSlider.setValue(*audioParam);
    mMidDWSPSlider.addListener(this);
    addAndMakeVisible(mMidDWSPSlider);

    
    // HIGH DWSP

    audioParam = (juce::AudioParameterFloat*)params.getUnchecked(6);

    mHighDWSPSlider.setSliderStyle(juce::Slider::SliderStyle::RotaryHorizontalVerticalDrag);
    mHighDWSPSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 40, 25);
    mHighDWSPSlider.setTextValueSuffix(" %");
    mHighDWSPSlider.setDoubleClickReturnValue(true, 20);
    mHighDWSPSlider.setNumDecimalPlacesToDisplay(0);
    mHighDWSPSlider.setRange(audioParam->range.start, audioParam->range.end);
    mHighDWSPSlider.setValue(*audioParam);
    mHighDWSPSlider.addListener(this);
    addAndMakeVisible(mHighDWSPSlider);

    // LOW GAIN

    audioParam = (juce::AudioParameterFloat*)params.getUnchecked(7);

    mLowDBSlider.setSliderStyle(juce::Slider::SliderStyle::LinearVertical);
    mLowDBSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 70, 35);
    mLowDBSlider.setTextValueSuffix(" dB");
    mLowDBSlider.setDoubleClickReturnValue(true, 0);
    mLowDBSlider.setNumDecimalPlacesToDisplay(0);
    mLowDBSlider.setRange(audioParam->range.start, audioParam->range.end);
    mLowDBSlider.setSkewFactorFromMidPoint(-10);
    mLowDBSlider.setValue(*audioParam);
    mLowDBSlider.addListener(this);
    addAndMakeVisible(mLowDBSlider);

    // MID GAIN

    audioParam = (juce::AudioParameterFloat*)params.getUnchecked(8);

    mMidDBSlider.setSliderStyle(juce::Slider::SliderStyle::LinearVertical);
    mMidDBSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 70, 35);
    mMidDBSlider.setTextValueSuffix(" dB");
    mMidDBSlider.setDoubleClickReturnValue(true, 0);
    mMidDBSlider.setNumDecimalPlacesToDisplay(0);
    mMidDBSlider.setRange(audioParam->range.start, audioParam->range.end);
    mMidDBSlider.setSkewFactorFromMidPoint(-10);
    mMidDBSlider.setValue(*audioParam);
    mMidDBSlider.addListener(this);
    addAndMakeVisible(mMidDBSlider);

    // HIGH GAIN

    audioParam = (juce::AudioParameterFloat*)params.getUnchecked(9);

    mHighDBSlider.setSliderStyle(juce::Slider::SliderStyle::LinearVertical);
    mHighDBSlider.setTextBoxStyle(juce::Slider::TextBoxBelow, false, 50, 30);
    mHighDBSlider.setTextValueSuffix(" dB");
    mHighDBSlider.setDoubleClickReturnValue(true, 0);
    mHighDBSlider.setNumDecimalPlacesToDisplay(0);
    mHighDBSlider.setRange(audioParam->range.start, audioParam->range.end);
    mHighDBSlider.setSkewFactorFromMidPoint(-10);
    mHighDBSlider.setValue(*audioParam);
    mHighDBSlider.addListener(this);
    addAndMakeVisible(mHighDBSlider);

    // LOW SOLO ENABLE

    //audioParam = (juce::AudioParameterFloat*)params.getUnchecked(10);
    mLowSoloEnable.setButtonText("Solo");
    mLowSoloEnable.setClickingTogglesState(true);
    mLowSoloEnable.addListener(this);
    addAndMakeVisible(mLowSoloEnable);

    // MID SOLO ENABLE

    mMidSoloEnable.setButtonText("Solo");
    mMidSoloEnable.setClickingTogglesState(true);
    mMidSoloEnable.addListener(this);
    addAndMakeVisible(mMidSoloEnable);

    // HIGH SOLO ENABLE

    mHighSoloEnable.setButtonText("Solo");
    mHighSoloEnable.setClickingTogglesState(true);
    mHighSoloEnable.addListener(this);
    addAndMakeVisible(mHighSoloEnable);

}

BrianTPFinalDigitalHellAudioProcessorEditor::~BrianTPFinalDigitalHellAudioProcessorEditor()
{
}

//==============================================================================
void BrianTPFinalDigitalHellAudioProcessorEditor::paint (juce::Graphics& g)
{


    // (Our component is opaque, so we must completely fill the background with a solid colour)
    g.fillAll (getLookAndFeel().findColour (juce::ResizableWindow::backgroundColourId));

    g.setColour (Colours::lightgrey);
    g.setFont(Font("Courier New", 25.0f, Font::bold));

    Rectangle<float> logoRectangle = Rectangle<float>::Rectangle(0, 0, 600, 100);
    g.drawImage(logo, logoRectangle);
    
    // DIVIDERS AND LARGE LABELS

    Line<float> lowDividerVert(Point<float>(300, 100), Point<float>(300, 575));
    g.drawLine(lowDividerVert);

    Line<float> highDividerVert(Point<float>(575, 100), Point<float>(575, 575));
    g.drawLine(highDividerVert);

    Line<float> lowDividerHorz(Point<float>(0, 140), Point<float>(875, 140));
    g.drawLine(lowDividerHorz);

    Line<float> highDividerHorz(Point<float>(0, 100), Point<float>(875, 100));
    g.drawLine(highDividerHorz);

    juce::Rectangle<int> lowLabelRectangle = juce::Rectangle<int>::Rectangle(0, 100, 300, 40);
    g.drawFittedText("LOW", lowLabelRectangle, juce::Justification::centred, 1);

    juce::Rectangle<int> midLabelRectangle = juce::Rectangle<int>::Rectangle(300, 100, 275, 40);
    g.drawFittedText("MID", midLabelRectangle, juce::Justification::centred, 1);

    juce::Rectangle<int> highLabelRectangle = juce::Rectangle<int>::Rectangle(575, 100, 300, 40);
    g.drawFittedText("HIGH", highLabelRectangle, juce::Justification::centred, 1);

    // SMALLER LABLES

    // CUTOFFS

    g.setColour(Colours::mediumpurple);
    g.setFont(Font("Courier New", 40.0f, Font::plain));

    juce::Rectangle<int> lowCutoffLabel = juce::Rectangle<int>::Rectangle(15, 160, 150, 100);
    g.drawFittedText("Cutoff:", lowCutoffLabel, Justification::left, 1);

    /*juce::Rectangle<int> midCutoffLabel = juce::Rectangle<int>::Rectangle(315, 140, 100, 100);
    g.drawFittedText("Cutoff:", midCutoffLabel, Justification::left, 1);*/

    juce::Rectangle<int> highCutoffLabel = juce::Rectangle<int>::Rectangle(590, 150, 150, 100);
    g.drawFittedText("Cutoff:", highCutoffLabel, Justification::left, 1);

    g.setColour(Colours::lightgrey);
    g.setFont(Font("Arial", 17.0f, Font::plain));

    // CUTOFF FILTER LABELS

    Rectangle<int> lowCutoffHPFLabel = Rectangle<int>::Rectangle(160, 235, 115, 50);
    g.drawFittedText("Hi Pass", lowCutoffHPFLabel, Justification::centred, 1);
    
    Rectangle<int> midCutoffHPFLabel = Rectangle<int>::Rectangle(315, 235, 115, 50);
    g.drawFittedText("Hi Pass", midCutoffHPFLabel, Justification::centred, 1);

    Rectangle<int> midCutoffLPFLabel = Rectangle<int>::Rectangle(430, 235, 115, 50);
    g.drawFittedText("Lo Pass", midCutoffLPFLabel, Justification::centred, 1);

    Rectangle<int> highCutoffLPFLabel = Rectangle<int>::Rectangle(735, 235, 115, 50);
    g.drawFittedText("Lo Pass", highCutoffLPFLabel, Justification::centred, 1);

    g.setFont(Font("Arial", 11.0f, Font::plain));

    Rectangle<int> lowCutoffLPFLabel = Rectangle<int>::Rectangle(315, 255, 115, 50);
    g.drawFittedText("+ Lo Pass (LOW)", lowCutoffLPFLabel, Justification::centred, 1);

    Rectangle<int> highCutoffHPFLabel = Rectangle<int>::Rectangle(430, 255, 115, 50);
    g.drawFittedText("+ Hi Pass (HIGH)", highCutoffHPFLabel, Justification::centred, 1);

    // DWSP

    g.setColour(Colours::mediumpurple);
    g.setFont(Font("Courier New", 30.0f, Font::plain));

    Rectangle<int> lowDWSPLabel = Rectangle<int>::Rectangle(-25, 275, 300, 100);
    g.drawFittedText("Downsample:", lowDWSPLabel, Justification::centred, 1);

    Rectangle<int> midDWSPLabel = Rectangle<int>::Rectangle(275, 275, 275, 100);
    g.drawFittedText("Downsample:", midDWSPLabel, Justification::centred, 1);

    Rectangle<int> highDWSPLabel = Rectangle<int>::Rectangle(550, 275, 300, 100);
    g.drawFittedText("Downsample:", highDWSPLabel, Justification::centred, 1);

    // GAIN

    g.setFont(Font("Courier New", 32.0f, Font::plain));

    Rectangle<int> lowGainLabel = Rectangle<int>::Rectangle(125, 510, 100, 100);
    g.drawFittedText("Gain:", lowGainLabel, Justification::left, 1);

    Rectangle<int> midGainLabel = Rectangle<int>::Rectangle(400, 510, 100, 100);
    g.drawFittedText("Gain:", midGainLabel, Justification::left, 1);

    Rectangle<int> highGainLabel = Rectangle<int>::Rectangle(700, 510, 100, 100);
    g.drawFittedText("Gain:", highGainLabel, Justification::left, 1);


    
    // OBJECTS

    // DWSP KNOB



}

void BrianTPFinalDigitalHellAudioProcessorEditor::resized()
{
    // This is generally where you'll want to lay out the positions of any
    // subcomponents in your editor..

    // FREQUENCIES

    mLowHPFcSlider.setBounds(160, 140, 115, 115);
    mLowLPMidHPFcSlider.setBounds(315, 140, 115, 115);
    mMidLPHighHPFcSlider.setBounds(430, 140, 115, 115);
    mHighLPFcSlider.setBounds(735, 140, 115, 115);

    // DWSP KNOBS

    Rectangle<int> lowDWSPKnobRectangle = getSquare(125, 425, 175);
    mLowDWSPSlider.setBounds(lowDWSPKnobRectangle);

    Rectangle<int> midDWSPKnobRectangle = getSquare(412, 425, 175);
    mMidDWSPSlider.setBounds(midDWSPKnobRectangle);

    Rectangle<int> highDWSPKnobRectangle = getSquare(700, 425, 175);
    mHighDWSPSlider.setBounds(highDWSPKnobRectangle);

    // GAIN SLIDERS

    mLowDBSlider.setBounds(225, 375, 50, 200);
    mMidDBSlider.setBounds(500, 375, 50, 200);
    mHighDBSlider.setBounds(800, 375, 50, 200);

    // SOLO BUTTONS

    mLowSoloEnable.setBounds(15, 535, 100, 50);
    mMidSoloEnable.setBounds(315, 535, 100, 50);
    mHighSoloEnable.setBounds(590, 535, 100, 50);

}

void BrianTPFinalDigitalHellAudioProcessorEditor::sliderValueChanged(Slider* slider) {

    auto& params = processor.getParameters();
    AudioParameterFloat* audioParam;

    if (slider == &mLowHPFcSlider) {
        audioParam = (AudioParameterFloat*)params.getUnchecked(0);
        *audioParam = mLowHPFcSlider.getValue();
    }

    else if (slider == &mLowLPMidHPFcSlider) {
        audioParam = (AudioParameterFloat*)params.getUnchecked(1);
        *audioParam = mLowLPMidHPFcSlider.getValue();
    }
    
    else if (slider == &mMidLPHighHPFcSlider) {
        audioParam = (AudioParameterFloat*)params.getUnchecked(2);
        *audioParam = mMidLPHighHPFcSlider.getValue();
    }

    else if (slider == &mHighLPFcSlider) {
        audioParam = (AudioParameterFloat*)params.getUnchecked(3);
        *audioParam = mHighLPFcSlider.getValue();
    }

    else if (slider == &mLowDWSPSlider) {
        audioParam = (AudioParameterFloat*)params.getUnchecked(4);
        *audioParam = mLowDWSPSlider.getValue();
    }

    else if (slider == &mMidDWSPSlider) {
        audioParam = (AudioParameterFloat*)params.getUnchecked(5);
        *audioParam = mMidDWSPSlider.getValue();
    }

    else if (slider == &mHighDWSPSlider) {
        audioParam = (AudioParameterFloat*)params.getUnchecked(6);
        *audioParam = mHighDWSPSlider.getValue();
    }

    else if (slider == &mLowDBSlider) {
        audioParam = (AudioParameterFloat*)params.getUnchecked(7);
        *audioParam = mLowDBSlider.getValue();
    }

    else if (slider == &mMidDBSlider) {
        audioParam = (AudioParameterFloat*)params.getUnchecked(8);
        *audioParam = mMidDBSlider.getValue();
    }

    else if (slider == &mHighDBSlider) {
        audioParam = (AudioParameterFloat*)params.getUnchecked(9);
        *audioParam = mHighDBSlider.getValue();
    }

}

void BrianTPFinalDigitalHellAudioProcessorEditor::buttonClicked(Button* button) {

    auto& params = processor.getParameters();
    AudioParameterFloat* audioParam;

    if (button == &mLowSoloEnable) {
        bool lowSolo = mLowSoloEnable.getToggleStateValue().getValue();
        audioProcessor.soloLowBand(lowSolo);
        mMidSoloEnable.setToggleState(false, dontSendNotification);
        mHighSoloEnable.setToggleState(false, dontSendNotification);
    }

    else if (button == &mMidSoloEnable) {
        bool midSolo = mMidSoloEnable.getToggleStateValue().getValue();
        audioProcessor.soloMidBand(midSolo);
        mLowSoloEnable.setToggleState(false, dontSendNotification);
        mHighSoloEnable.setToggleState(false, dontSendNotification);
    }

    if (button == &mHighSoloEnable) {
        bool highSolo = mHighSoloEnable.getToggleStateValue().getValue();
        audioProcessor.soloHighBand(highSolo);
        mLowSoloEnable.setToggleState(false, dontSendNotification);
        mMidSoloEnable.setToggleState(false, dontSendNotification);
    }

}

