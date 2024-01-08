#include <string>
#include "PongRosSdl.h"
#include "../../pong_ros_core/include/Constants.h"

using namespace pong_ros_constants;

class PongVisualization {
  private:
    double leftPaddlePosition  = 300; /// Vertical position of right bat. Units are up to the project group
    double rightPaddlePosition = 300; /// Vertical position of right bat. Units are up to the project group
    double ballPositionX = 0; /// Horizontal position of the (center of) the ball. Units are up to the project group
    double ballPositionY = 0; /// Vertical position of the (center of) the ball. Units are up to the project group
    std::string fieldText = "0 - 0";

  public:
    /** Constructor
     * Default
     */
    PongVisualization ():
        pong_ros_sdl(SCREEN_TITLE, SCREEN_WIDTH, SCREEN_HEIGHT, FONT_SIZE)
    {}

    /** Set the vertical position of the left bat.
     * Units are up to the project group.
     */
    void setLeftPaddlePosition(double v);

    /** Set the vertical position of the right bat.
     * Units are up to the project group.
     */
    void setRightPaddlePosition(double v);
    
    /** Set the horizontal and vertical position of the (center of) the ball.
     * Units are up to the project group.
     */
    void setBallPosition(double x_, double y_);

    /** Set the text to display in the middle of the field.
     *  Use \n in the string for a new line. You can use this to show the score, "0-4"
     *  or any other text, "Game over".
     *  The text is shown until a new text is set.
     *  To remove the text, send an empty string, "".
     */
    void setFieldText(std::string s_);

    /** Draw the current state of all objects on the screen. Just call this function regularly and 
     * you'll be fine...
     */
    void render();

    /** The screen canvas. Public so that you can
     * interact with from outside the PongVisualization class.
     */
    PongRosSdl pong_ros_sdl;
};
