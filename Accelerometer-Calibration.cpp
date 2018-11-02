class accelParam {
    private :
        double det(int dim, double mat[6][6]){
            double count = 0;
            if (dim == 2){
                return mat[0][0] * mat[1][1] - mat[0][1] * mat[1][0];
            }
            for(int i = 0; i < dim; i++){
                double matP[6][6];
                int sign = i%2==0?1:-1;
                double cofactor = sign*mat[0][i];
                int index = 0;
                for(int j = 0; j < dim; j++) {
                    if (i==j) {
                        continue;
                    }
                    for (int k = 1; k < dim; k++){
                        matP[k-1][index] = mat[k][j];
                    }
                    index ++;
                }
                count += cofactor*det(dim-1, matP);
            }
            return count;
        }
        double calibrationParam[4][3] = {{0,0,0},{0,0,0},{0,0,0},{0,0,0}};
        const double positions[6][3] = {{0,0,1},{0,0,-1},{0,1,0},{0,-1,0},{1,0,0},{-1,0,0}};
        double measuredValues[6][4];
        double measuredValuesTx1[4][4];
        double setParam(){
            for (int i = 0; i < 4; i++){
                for (int j = 0; i < 4; i++){
                    dotSum = 0;
                    for (int k = 0; k < 6; i++){
                        dotSum += measuredValues[k][i] * measuredValues[k][j];
                    }
                    measuredValuesTx1[i][j];
                }
            }
            
        }
    public:
        double calculatePitch(){
        }
        double calibrateAccelerometer(){
        }
        
}
