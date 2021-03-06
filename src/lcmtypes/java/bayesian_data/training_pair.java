/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package bayesian_data;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class training_pair implements lcm.lcm.LCMEncodable
{
    public double training_x_[];
    public double training_y_;
 
    public training_pair()
    {
        training_x_ = new double[5];
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x0e86f2ac9014a632L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(bayesian_data.training_pair.class))
            return 0L;
 
        classes.add(bayesian_data.training_pair.class);
        long hash = LCM_FINGERPRINT_BASE
            ;
        classes.remove(classes.size() - 1);
        return (hash<<1) + ((hash>>63)&1);
    }
 
    public void encode(DataOutput outs) throws IOException
    {
        outs.writeLong(LCM_FINGERPRINT);
        _encodeRecursive(outs);
    }
 
    public void _encodeRecursive(DataOutput outs) throws IOException
    {
        for (int a = 0; a < 5; a++) {
            outs.writeDouble(this.training_x_[a]); 
        }
 
        outs.writeDouble(this.training_y_); 
 
    }
 
    public training_pair(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public training_pair(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static bayesian_data.training_pair _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        bayesian_data.training_pair o = new bayesian_data.training_pair();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.training_x_ = new double[(int) 5];
        for (int a = 0; a < 5; a++) {
            this.training_x_[a] = ins.readDouble();
        }
 
        this.training_y_ = ins.readDouble();
 
    }
 
    public bayesian_data.training_pair copy()
    {
        bayesian_data.training_pair outobj = new bayesian_data.training_pair();
        outobj.training_x_ = new double[(int) 5];
        System.arraycopy(this.training_x_, 0, outobj.training_x_, 0, 5); 
        outobj.training_y_ = this.training_y_;
 
        return outobj;
    }
 
}

