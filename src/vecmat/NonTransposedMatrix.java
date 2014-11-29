package vecmat;

public final class NonTransposedMatrix extends Matrix
{
    NonTransposedMatrix(double[][] data)
    {
        super(data, data.length, 0 < data.length ? data[0].length : 0);
        trMat = new TransposedMatrix(this);
    }
    
    //--------------------------------------------------------------------------

    @Override
    public double get(int i, int j)
    {
        assert (0 <= i && i < rows());
        assert (0 <= j && j < cols());
        return array()[i][j];
    }
    
    @Override
    public void set(int i, int j, double value)
    {
        assert (0 <= i && i < rows());
        assert (0 <= j && j < cols());
        array()[i][j] = value;
    }
    
    //--------------------------------------------------------------------------
        
    @Override
    public Matrix T()
    {
        return trMat;
    }
    
    //--------------------------------------------------------------------------
    
    private final TransposedMatrix trMat;
}
