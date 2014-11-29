package vecmat;

import junit.framework.TestCase;

/**
 * Enable the implementation assertions.
 */
public class AssertionBaseTest extends TestCase
{
    public AssertionBaseTest(String name) { super(name); }

    static
    {
        ClassLoader.getSystemClassLoader().setDefaultAssertionStatus(true);
    }
}
