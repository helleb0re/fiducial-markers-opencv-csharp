namespace comparing_fiducial_markers;

public class FiducialMarker
{

    public string Name { get; }
    public float Size { get; }
    
    public Dictionary<Test, Positions> TestResults;

    public FiducialMarker(string libName, float size)
    {
        Name = libName;
        Size = size;

        TestResults = new Dictionary<Test, Positions>();
    }
}