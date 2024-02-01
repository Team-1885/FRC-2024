package frc.common;


import frc.robot.hardware.vendors.firstparties.ABC;
import frc.robot.hardware.vendors.firstparties.ABC.Distance;

public class Field2024 {

    public enum Distances{
        TARMAC_TO_HUB(84.25),
        HUB_RING_DIAMETER(306),
        BALL_TO_HUB(126.5),
        TARGETTING_OFFSET(21.0);

        public Distance mDistance;
        public ABC abc = new ABC();
        Distances(double pDist) {
        
          mDistance = abc.getDistance().fromInches(pDist);
        }
    }

    /**
     * Any trackable field component for 2020's game.
     */
    public enum FieldElement implements IFieldComponent {
        //Do not switch order
        NULL (0, 0, 0),
        HUB_UPPER (104,48, 1),
        TARGET_ZOOM (0d,0d),
        RED_BALL (0 , 0, 3),
        BLUE_BALL (0, 0, 4),
        CAMERA (0,0, 0);

        // This is done in inches -- straight from game manual
        private final double height;
        private final double width;
        private int pipeline;

        // Limelight-based FieldElement
        FieldElement(double pHeight, double pWidth, int pPipeline){
            width = pWidth;
            height = pHeight;
            pipeline = pPipeline;
        }

        // Limelight-based FieldElement
        FieldElement(double pHeight, double pWidth){
            width = pWidth;
            height = pHeight;
            pipeline = 1;
        }

        @Override
        public double width() { return width; }

        @Override
        public double height() {
          return height;
        }

        @Override
        public int pipeline() {
          return pipeline;
        }

        public int id() { return ordinal(); }
    }

    public static void main(String[] args) {
        for(FieldElement anElement: FieldElement.values()) {
            System.out.println(anElement.name()+" id: " + anElement.id()+", pipeline: "+ anElement.pipeline());
        }
    }
}