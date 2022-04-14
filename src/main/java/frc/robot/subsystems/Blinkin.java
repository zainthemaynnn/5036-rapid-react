package frc.robot.subsystems;

import java.util.PriorityQueue;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;

// THEY CALLED ME A MADMAN -- zain 2022-04-12
public class Blinkin implements Subsystem  {
    private interface Pattern {
        public double value();
    };

    public static enum FixedPalette implements Pattern {
        RAINBOW_RAINBOW(-.99),
        RAINBOW_PARTY(-.97),
        RAINBOW_OCEAN(-.95),
        RAINBOW_LAVE(-.93),
        RAINBOW_FOREST(-.91),
        RAINBOW_GLITTER(-.89),
        CONFETTI(-.87),
        SHOT_RED(-.85),
        SHOT_BLUE(-.83),
        SHOT_WHITE(-.81),
        SINELON_RAINBOW(-.79),
        SINELON_PARTY(-.77),
        SINELON_OCEAN(-.75),
        SINELON_LAVA(-.73),
        SINELON_FOREST(-.71),
        BPM_RAINBOW(-.69),
        BPM_PARTY(-.67),
        BPM_OCEAN(-.65),
        BPM_LAVA(-.63),
        BPM_FOREST(-.61),
        FIRE_MEDIUM(-.59),
        FIRE_LARGE(-.57),
        TWINKLES_RAINBOW(-.55),
        TWINKLES_PARTY(-.53),
        TWINKLES_OCEAN(-.51),
        TWINKLES_LAVA(-.49),
        TWINKLES_FOREST(-.47),
        COLOR_WAVES_RAINBOW(-.45),
        COLOR_WAVES_PARTY(-.43),
        COLOR_WAVES_OCEAN(-.41),
        COLOR_WAVES_LAVA(-.39),
        COLOR_WAVES_FOREST(-.37),
        LARSON_SCANNER_RED(-.35),
        LARSON_SCANNER_GRAY(-.33),
        LIGHT_CHASE_RED(-.31),
        LIGHT_CHASE_BLUE(-.29),
        LIGHT_CHASE_GRAY(-.27),
        HEARTBEAT_RED(-.25),
        HEARTBEAT_BLUE(-.23),
        HEARTBEAT_WHITE(-.21),
        HEARTBEAT_GRAY(-.19),
        BREATH_RED(-.17),
        BREATH_BLUE(-.15),
        BREATH_GRAY(-.13),
        STROBE_RED(-.11),
        STROBE_BLUE(-.09),
        STROBE_GOLD(-.07),
        STROBE_WHITE(-.05);

        private double value;
        private FixedPalette(double value) {
            this.value = value;
        }
        public double value() {
            return value;
        }
    }

    private static enum Color1 implements Pattern {
        BLEND_TO_BLACK(-.03),
        LARSON_SCANNER(-.01),
        LIGHT_CHASE(+.01),
        HEARTBEAT_SLOW(+.03),
        HEARTBEAT_MEDIUM(+.05),
        HEARTBEAT_FAST(+.07),
        BREATH_SLOW(+.09),
        BREATH_FAST(+.11),
        SHOT(+.13),
        STROBE(+.15);

        private double value;
        private Color1(double value) {
            this.value = value;
        }
        public double value() {
            return value;
        }
    }

    private static enum Color2 implements Pattern {
        BLEND_TO_BLACK(+.17),
        LARSON_SCANNER(+.19),
        LIGHT_CHASE(+.21),
        HEARTBEAT_SLOW(+.23),
        HEARTBEAT_MEDIUM(+.25),
        HEARTBEAT_FAST(+.27),
        BREATH_SLOW(+.29),
        BREATH_FAST(+.31),
        SHOT(+.33),
        STROBE(+.35);

        private double value;
        private Color2(double value) {
            this.value = value;
        }
        public double value() {
            return value;
        }
    }

    private static enum ColorMixed implements Pattern {
        SPARKLE_1_ON_2(+.37),
        SPARKLE_2_ON_1(+.39),
        COLOR_GRADIENT(+.41),
        BPM(+.43),
        BLEND_1_TO_2(+.45),
        BLEND(+.47),
        NO_BLEND_1_TO_2(+.49),
        TWINKLES(+.51),
        COLOR_WAVES(+.53),
        SINELON(+.55);

        private double value;
        private ColorMixed(double value) {
            this.value = value;
        }
        public double value() {
            return value;
        }
    }

    private static enum Solid implements Pattern {
        HOT_PINK(+.57),
        DARK_RED(+.59),
        RED(+.61),
        RED_ORANGE(+.63),
        ORANGE(+.65),
        GOLD(+.67),
        LAWN_GREEN(+.71),
        LINE(+.73),
        DARK_GREEN(+.75),
        GREEN(+.77),
        BLUE_GREEN(+.79),
        AQUA(+.81),
        SKY_BLUE(+.83),
        DARK_BLUE(+.85),
        BLUE(+.87),
        BLUE_VIOLET(+.89),
        VIOLET(+.91),
        WHITE(+.93),
        GRAY(+.95),
        DARK_GRAY(+.97),
        BLACK(+.99);

        private double value;
        private Solid(double value) {
            this.value = value;
        }
        public double value() {
            return value;
        }
    }

    public static enum BlinkinColor {
        DEFAULT (Solid.BLACK),
        OFF     (Solid.BLACK),
        BOOT    (Color2.STROBE),
        INTAKE  (Solid.GREEN),
        SHOOT   (Color2.STROBE);

        private double value;

        private BlinkinColor(Pattern p) {
            value = p.value();
        }

        public double value() {
            return value;
        }
    }

    private Spark blinkin;
    private PriorityQueue<BlinkinColor> queue;

    // https://www.revrobotics.com/content/docs/REV-11-1105-UM.pdf 
    // for all the LED Information

    public Blinkin(int port) { 
        blinkin = new Spark(port);
        queue = new PriorityQueue<>(BlinkinColor::compareTo);
    }

    public void addColor(BlinkinColor color) {
        queue.add(color);
    }

    public void removeColor(BlinkinColor color) {
        queue.remove(color);
    }

    public boolean containsColor(BlinkinColor color) {
        return queue.contains(color);
    }

    public void display() {
        if (!queue.isEmpty()) {
            blinkin.set(queue.peek().value());
        } else {
            blinkin.set(BlinkinColor.DEFAULT.value());
        }

        SmartDashboard.putNumber("Size", queue.size());
        SmartDashboard.putNumber("Blinkin", blinkin.get());
    }
}
