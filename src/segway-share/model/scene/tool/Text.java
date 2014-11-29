package model.scene.tool;

import java.awt.Font;

import model.scene.Color;

/**
 * Text on the top of the whole scene view.
 * 
 * Negative coordinates are measured from the right side / top of the canvas
 * while positive ones are from the left side / bottom respectively.
 */
public class Text extends SceneTool
{
    public Text(float x, float y)
    {
        setXY(x, y);
    }
    
    //--------------------------------------------------------------------------
    
    public String text() { return text; }
    public synchronized void setText(String text)
    { this.text = text; }
    
    public Color color() { return color; }
    public synchronized void setColor(Color color)
    { this.color = color; }
    
    public Font font() { return font; }
    public synchronized void setFont(Font font)
    { this.font = font; }
    
    public float x() { return x; }
    public float y() { return y; }    
    public synchronized void setXY(float x, float y)
    { this.x = x; this.y = y; }
    
    //--------------------------------------------------------------------------
    
    private float x, y;
    
    private String text = "";
    private Font font = new Font(Font.MONOSPACED, Font.BOLD, 16);
    private Color color = new Color(200f/255f, 200f/255f, 200f/255f);
}
