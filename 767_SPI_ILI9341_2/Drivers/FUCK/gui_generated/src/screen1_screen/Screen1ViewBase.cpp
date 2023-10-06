/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#include <gui_generated/screen1_screen/Screen1ViewBase.hpp>
#include <touchgfx/canvas_widget_renderer/CanvasWidgetRenderer.hpp>
#include <touchgfx/Color.hpp>
#include <texts/TextKeysAndLanguages.hpp>

Screen1ViewBase::Screen1ViewBase()
{
    touchgfx::CanvasWidgetRenderer::setupBuffer(canvasBuffer, CANVAS_BUFFER_SIZE);
    
    __background.setPosition(0, 0, 480, 272);
    __background.setColor(touchgfx::Color::getColorFromRGB(0, 0, 0));
    add(__background);

    box1.setPosition(0, 0, 480, 272);
    box1.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    add(box1);

    textArea1.setXY(76, 55);
    textArea1.setColor(touchgfx::Color::getColorFromRGB(0, 0, 0));
    textArea1.setLinespacing(0);
    textArea1.setTypedText(touchgfx::TypedText(T___SINGLEUSE_AE5B));
    add(textArea1);

    textArea1_2.setXY(384, 110);
    textArea1_2.setColor(touchgfx::Color::getColorFromRGB(0, 0, 0));
    textArea1_2.setLinespacing(0);
    textArea1_2.setTypedText(touchgfx::TypedText(T___SINGLEUSE_FY12));
    add(textArea1_2);

    textArea1_2_1.setXY(384, 166);
    textArea1_2_1.setColor(touchgfx::Color::getColorFromRGB(0, 0, 0));
    textArea1_2_1.setLinespacing(0);
    textArea1_2_1.setTypedText(touchgfx::TypedText(T___SINGLEUSE_AMQI));
    add(textArea1_2_1);

    textArea1_2_1_1.setXY(384, 216);
    textArea1_2_1_1.setColor(touchgfx::Color::getColorFromRGB(0, 0, 0));
    textArea1_2_1_1.setLinespacing(0);
    textArea1_2_1_1.setTypedText(touchgfx::TypedText(T___SINGLEUSE_2259));
    add(textArea1_2_1_1);

    textArea1_1.setXY(332, 55);
    textArea1_1.setColor(touchgfx::Color::getColorFromRGB(0, 0, 0));
    textArea1_1.setLinespacing(0);
    textArea1_1.setTypedText(touchgfx::TypedText(T___SINGLEUSE_C1F3));
    add(textArea1_1);

    circle1.setPosition(221, 25, 80, 80);
    circle1.setCenter(40, 40);
    circle1.setRadius(30);
    circle1.setLineWidth(0);
    circle1.setArc(0, 360);
    circle1Painter.setColor(touchgfx::Color::getColorFromRGB(255, 0, 238));
    circle1.setPainter(circle1Painter);
    add(circle1);

    circle2.setPosition(0, 0, 80, 80);
    circle2.setCenter(40, 40);
    circle2.setRadius(40);
    circle2.setLineWidth(0);
    circle2.setArc(0, 360);
    circle2Painter.setColor(touchgfx::Color::getColorFromRGB(255, 255, 255));
    circle2.setPainter(circle2Painter);
    add(circle2);

    circle3.setPosition(76, 80, 80, 80);
    circle3.setCenter(40, 40);
    circle3.setRadius(21.4f);
    circle3.setLineWidth(0);
    circle3.setArc(0, 360);
    circle3Painter.setColor(touchgfx::Color::getColorFromRGB(255, 0, 0));
    circle3.setPainter(circle3Painter);
    add(circle3);

    circle3_1.setPosition(76, 136, 80, 80);
    circle3_1.setCenter(40, 40);
    circle3_1.setRadius(21.4f);
    circle3_1.setLineWidth(0);
    circle3_1.setArc(0, 360);
    circle3_1Painter.setColor(touchgfx::Color::getColorFromRGB(17, 255, 0));
    circle3_1.setPainter(circle3_1Painter);
    add(circle3_1);

    circle3_1_1.setPosition(76, 192, 80, 80);
    circle3_1_1.setCenter(40, 40);
    circle3_1_1.setRadius(21.4f);
    circle3_1_1.setLineWidth(0);
    circle3_1_1.setArc(0, 360);
    circle3_1_1Painter.setColor(touchgfx::Color::getColorFromRGB(0, 8, 255));
    circle3_1_1.setPainter(circle3_1_1Painter);
    add(circle3_1_1);

    box2_2.setPosition(156, 216, 200, 20);
    box2_2.setColor(touchgfx::Color::getColorFromRGB(145, 149, 255));
    add(box2_2);

    box2_1.setPosition(156, 110, 200, 20);
    box2_1.setColor(touchgfx::Color::getColorFromRGB(255, 120, 120));
    add(box2_1);

    box2_1_2.setPosition(156, 164, 200, 20);
    box2_1_2.setColor(touchgfx::Color::getColorFromRGB(146, 255, 138));
    add(box2_1_2);

    box2_1_1.setPosition(156, 110, 170, 20);
    box2_1_1.setColor(touchgfx::Color::getColorFromRGB(255, 0, 0));
    add(box2_1_1);

    box2.setPosition(156, 164, 80, 20);
    box2.setColor(touchgfx::Color::getColorFromRGB(17, 255, 0));
    add(box2);

    box2_3.setPosition(156, 216, 140, 20);
    box2_3.setColor(touchgfx::Color::getColorFromRGB(0, 8, 255));
    add(box2_3);
}

Screen1ViewBase::~Screen1ViewBase()
{
    touchgfx::CanvasWidgetRenderer::resetBuffer();
}

void Screen1ViewBase::setupScreen()
{

}
