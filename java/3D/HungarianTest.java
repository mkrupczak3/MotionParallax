import MotionParallax.HungarianAlgorithm;

public class HungarianTest {
    public static void main(String[] args) {

        int[][] dataMatrix = {
            {70, 40, 99999, 99999},
            {65, 60, 99999, 99999},
            {30, 45, 99999, 99999},
            {25, 30, 99999, 99999}
        };

        //find optimal assignment
        HungarianAlgorithm ha = new HungarianAlgorithm(dataMatrix);
        int[][] assignment = ha.findOptimalAssignment();

        if (assignment.length > 0) {
            // print assignment
            for (int i = 0; i < assignment.length; i++) {
                System.out.print("Col" + assignment[i][0] + " => Row" + assignment[i][1] + " (" + dataMatrix[assignment[i][0]][assignment[i][1]] + ")");
                System.out.println();
            }
        } else {
            System.out.println("no assignment found!");
        }
    }
}
