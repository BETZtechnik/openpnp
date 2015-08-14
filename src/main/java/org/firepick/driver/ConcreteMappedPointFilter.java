package org.firepick.driver;

import java.io.IOException;
import java.io.Reader;

import org.firepick.gfilter.MappedPointFilter;

/**
 * MappedPointFilter's constructors are all protected for some reason. Not
 * sure why. So this is just to open that up so we can use it. 
 */
public class ConcreteMappedPointFilter extends MappedPointFilter {
    public ConcreteMappedPointFilter(Reader reader) throws IOException {
        super(null, reader);
    }
}
