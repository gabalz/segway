package visual.engine;

import java.awt.Color;
import java.awt.Font;
import java.awt.FontMetrics;
import java.awt.Graphics2D;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.nio.ByteBuffer;
import java.nio.IntBuffer;
import java.util.HashMap;

import org.lwjgl.BufferUtils;
import org.lwjgl.opengl.GL11;

/**
 * Visualization of AWT fonts.
 * Fonts has to be created during the GL initialization phase.
 */
public class EngineFont
{
    private EngineFont(Font font)
    {
        assert (fonts != null);
        this.font = font;
        buildAscii();
    }
    
    /** @return the AWT representation of the font */
    public Font awt() { return font; }

    //--------------------------------------------------------------------------
    
    public void glPrint(String text)
    {
        GL11.glTranslatef(0f, -maxAscent, 0f);
        /* // Indirect buffers are not supported by LWJGL.
         * GL11.glPushAttrib(GL11.GL_LIST_BIT);
         * GL11.glListBase(base);
         * GL11.glCallLists(ByteBuffer.wrap(text.getBytes()));
         * GL11.glPopAttrib();
         */        
        for (int i = 0; i < text.length(); ++i)
            GL11.glCallList(base + text.charAt(i));
        GL11.glBindTexture(GL11.GL_TEXTURE_2D, 0);
    }
    
    //--------------------------------------------------------------------------

    private void buildAscii()
    {
        final int chN = 128;
        
        // determining font metrics
        
        FontMetrics metrics;
        int maxHeight;
        {
            BufferedImage image =
                new BufferedImage(1, 1, BufferedImage.TYPE_BYTE_GRAY);
            Graphics2D g = (Graphics2D) image.getGraphics();
            g.setFont(font);
            metrics = g.getFontMetrics();
            
            maxAscent = metrics.getMaxAscent();
            maxHeight = maxAscent + metrics.getMaxDescent();
        }
        
        // creating character textures and display lists
        
        textureIDs = BufferUtils.createIntBuffer(chN);
        GL11.glGenTextures(textureIDs);
        
        base = GL11.glGenLists(chN);
        
        for (char ch = 0; ch < chN; ++ch)
        {
            final int chWidth = metrics.charWidth(ch);
            
            // drawing the character into an image
            final int imgW = nextP2(chWidth), imgH = nextP2(maxHeight);
            BufferedImage image =
                new BufferedImage(imgW, imgH, BufferedImage.TYPE_BYTE_GRAY);
            Graphics2D g = (Graphics2D) image.getGraphics();
            // g.setRenderingHint(RenderingHints.KEY_ANTIALIASING,
            //                    RenderingHints.VALUE_ANTIALIAS_ON);
            
            g.setBackground(TRANSPARENT_BLACK);
            g.clearRect(0, 0, imgW, imgH);
            g.setFont(font);
            g.setColor(OPAQUE_WHITE);
            g.drawString("" + ch, 0f, maxAscent);
            
            // creating the texture based on the image
            ByteBuffer buffer =
                BufferUtils.createByteBuffer(2 * imgW * imgH);
            {
                byte[] data =
                    ((DataBufferByte)image.getData().getDataBuffer()).getData();
                for (int i = 0; i < data.length; ++i)
                {
                    // same values for luminosity and alpha
                    buffer.put(data[i]); // luminosity
                    buffer.put(data[i]); // alpha
                }
                buffer.flip();
            }
            
            GL11.glBindTexture(GL11.GL_TEXTURE_2D, textureIDs.get(ch));
            
            GL11.glTexParameteri(GL11.GL_TEXTURE_2D,
                                 GL11.GL_TEXTURE_MIN_FILTER,
                                 GL11.GL_LINEAR);
            GL11.glTexParameteri(GL11.GL_TEXTURE_2D,
                                 GL11.GL_TEXTURE_MAG_FILTER,
                                 GL11.GL_LINEAR);
            
            GL11.glTexImage2D(GL11.GL_TEXTURE_2D, 0, GL11.GL_RGBA,
                              imgW, imgH, 0, GL11.GL_LUMINANCE_ALPHA,
                              GL11.GL_UNSIGNED_BYTE, buffer);
            
            // creating the display list
            GL11.glNewList(base + ch, GL11.GL_COMPILE);
            GL11.glBindTexture(GL11.GL_TEXTURE_2D, textureIDs.get(ch));
            GL11.glBegin(GL11.GL_QUADS);
                GL11.glTexCoord2f(0f, 1f);
                GL11.glVertex2f(0f, 0f);
                GL11.glTexCoord2f(1f, 1f);
                GL11.glVertex2f(imgW, 0f);
                GL11.glTexCoord2f(1f, 0f);
                GL11.glVertex2f(imgW, imgH);
                GL11.glTexCoord2f(0f, 0f);
                GL11.glVertex2f(0f, imgH);                
            GL11.glEnd();
            GL11.glTranslatef(chWidth, 0f, 0f);
            GL11.glEndList();
        }
        
        GL11.glBindTexture(GL11.GL_TEXTURE_2D, 0);
    }
    
    private static int nextP2(int x)
    {
        int p = 1;
        while (p < x) p *= 2;
        return p;
    }
    
    //--------------------------------------------------------------------------
    
    private int base, maxAscent;
    private IntBuffer textureIDs;
    private final Font font;
    
    //--------------------------------------------------------------------------
    
    private static final Color OPAQUE_WHITE = new Color(1f, 1f, 1f, 1f);
    private static final Color TRANSPARENT_BLACK = new Color(0f, 0f, 0f, 0f);
    
    private static final HashMap<String, EngineFont> fonts =
        new HashMap<String, EngineFont>();
    
    public static EngineFont provideFont(Font font)
    {
        EngineFont eFont = fonts.get(font.toString());
        if (eFont == null) // create the font
        {
            eFont = new EngineFont(font);
            fonts.put(font.toString(), eFont);
        }
        return eFont;
    }
}
