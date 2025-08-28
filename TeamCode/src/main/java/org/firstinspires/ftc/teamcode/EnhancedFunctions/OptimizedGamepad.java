package org.firstinspires.ftc.teamcode.EnhancedFunctions;

import static org.firstinspires.ftc.teamcode.EnhancedFunctions.OptimizedGamepad.GAMEPAD_BOOLS.*;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;


public class OptimizedGamepad {

    private Gamepad gamepad;

    public Booleans booleans;
    public Floats floats;
    public Commands commands;

    public OptimizedGamepad(Gamepad gamepad) {

        this.gamepad = gamepad;

        booleans = new Booleans();
        floats = new Floats();
        commands = new Commands();
    }

    public class Booleans {

        public Booleans.WasPressed wasPressed;
        public Booleans.WasReleased wasReleased;

        public Booleans() {

            wasPressed = new WasPressed();
            wasReleased = new WasReleased();
        }

        public boolean a() { return gamepad.a; }
        public boolean b() { return gamepad.b; }
        public boolean x() { return gamepad.x; }
        public boolean y() { return gamepad.y; }

        public boolean circle() { return gamepad.circle; }
        public boolean cross() { return gamepad.cross; }
        public boolean square() { return gamepad.square; }
        public boolean triangle() { return gamepad.triangle; }

        public boolean dpad_up() { return gamepad.dpad_up; }
        public boolean dpad_down() { return gamepad.dpad_down; }
        public boolean dpad_left() { return gamepad.dpad_left; }
        public boolean dpad_right() { return gamepad.dpad_right; }

        public boolean left_bumper() { return gamepad.left_bumper; }
        public boolean right_bumper() { return gamepad.right_bumper; }

        public boolean left_trigger(float threshold) { return gamepad.left_trigger >= threshold; }
        public boolean right_trigger(float threshold) { return gamepad.right_trigger >= threshold; }

        public boolean back() { return gamepad.back; }

        public boolean main_button() { return gamepad.guide || gamepad.ps; }

        public boolean atRest() { return gamepad.atRest(); }

        public boolean hasJustBeenPressed(GAMEPAD_BOOLS bools) {

            if ((bools == A || bools == SQUARE) && gamepad.a && !gamepad.aWasPressed()) return true;
            if ((bools == B || bools == CIRCLE) && gamepad.b && !gamepad.bWasPressed()) return true;
            if ((bools == X || bools == CROSS) && gamepad.x && !gamepad.xWasPressed()) return true;
            if ((bools == Y || bools == TRIANGLE) && gamepad.y && !gamepad.yWasPressed()) return true;

            if (bools == MAIN_BUTTON && (gamepad.guide && !gamepad.guideWasPressed()) || (gamepad.ps && !gamepad.psWasPressed())) return true;

            if (bools == DPAD_UP && gamepad.dpad_up && !gamepad.dpadUpWasPressed()) return true;
            if (bools == DPAD_DOWN && gamepad.dpad_down && !gamepad.dpadDownWasPressed()) return true;
            if (bools == DPAD_LEFT && gamepad.dpad_left && !gamepad.dpadLeftWasPressed()) return true;
            if (bools == DPAD_RIGHT && gamepad.dpad_right && !gamepad.dpadRightWasPressed()) return true;

            if (bools == LB && gamepad.left_bumper && !gamepad.leftBumperWasPressed()) return true;
            if (bools == RB && gamepad.right_bumper && !gamepad.rightBumperWasPressed()) return true;

            return false;
        }

        public class WasPressed {

            public boolean aWasPressed() { return gamepad.aWasPressed(); }
            public boolean bWasPressed() { return gamepad.bWasPressed(); }
            public boolean xWasPressed() { return gamepad.xWasPressed(); }
            public boolean yWasPressed() { return gamepad.yWasPressed(); }

            public boolean circleWasPressed() { return gamepad.circleWasPressed(); }
            public boolean crossWasPressed() { return gamepad.crossWasPressed(); }
            public boolean squareWasPressed() { return gamepad.squareWasPressed(); }
            public boolean triangleWasPressed() { return gamepad.triangleWasPressed(); }

            public boolean dpadUpWasPressed() { return gamepad.dpadUpWasPressed(); }
            public boolean dpadDownWasPressed() { return gamepad.dpadDownWasPressed(); }
            public boolean dpadLeftWasPressed() { return gamepad.dpadLeftWasPressed(); }
            public boolean dpadRightWasPressed() { return gamepad.dpadRightWasPressed(); }

            public boolean leftBumperWasPressed() { return gamepad.leftBumperWasPressed(); }
            public boolean rightBumperWasPressed() { return gamepad.rightBumperWasPressed(); }

            public boolean backWasPressed() { return gamepad.backWasPressed(); }

            public boolean mainButtonWasPressed() { return gamepad.guideWasPressed() || gamepad.psWasPressed(); }

        }

        public class WasReleased {

            public boolean aWasReleased() { return gamepad.aWasReleased(); }
            public boolean bWasReleased() { return gamepad.bWasReleased(); }
            public boolean xWasReleased() { return gamepad.xWasReleased(); }
            public boolean yWasReleased() { return gamepad.yWasReleased(); }

            public boolean circleWasReleased() { return gamepad.circleWasReleased(); }
            public boolean crossWasReleased() { return gamepad.crossWasReleased(); }
            public boolean squareWasReleased() { return gamepad.squareWasReleased(); }
            public boolean triangleWasReleased() { return gamepad.triangleWasReleased(); }

            public boolean dpadUpWasReleased() { return gamepad.dpadUpWasReleased(); }
            public boolean dpadDownWasReleased() { return gamepad.dpadDownWasReleased(); }
            public boolean dpadLeftWasReleased() { return gamepad.dpadLeftWasReleased(); }
            public boolean dpadRightWasReleased() { return gamepad.dpadRightWasReleased(); }

            public boolean leftBumperWasReleased() { return gamepad.leftBumperWasReleased(); }
            public boolean rightBumperWasReleased() { return gamepad.rightBumperWasReleased(); }

            public boolean backWasReleased() { return gamepad.backWasReleased(); }

            public boolean mainButtonWasReleased() { return gamepad.guideWasReleased() || gamepad.psWasReleased(); }
        }

    }

    public enum GAMEPAD_BOOLS {
        A, B, X, Y,
        CIRCLE, CROSS, SQUARE, TRIANGLE,
        LB, RB,
        DPAD_UP, DPAD_DOWN, DPAD_LEFT, DPAD_RIGHT,
        MAIN_BUTTON
    }

    public class Floats {

        public float left_stick_y() { return gamepad.left_stick_y; }
        public float left_stick_x() { return gamepad.left_stick_x; }
        public float right_stick_y() { return gamepad.right_stick_y; }
        public float right_stick_x() { return gamepad.right_stick_x; }

        public float left_trigger() { return gamepad.left_trigger; }
        public float right_trigger() { return gamepad.right_trigger; }
    }

    public class Commands {

        public long timestamp() { return gamepad.timestamp; }

        public void stopRumble() { gamepad.stopRumble(); }
        public void rumble(int durationMs) { gamepad.rumble(durationMs); }

        public Gamepad.Type type() { return gamepad.type; }

        public GamepadUser getUser() { return gamepad.getUser(); }
    }
}