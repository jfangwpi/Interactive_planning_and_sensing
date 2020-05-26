/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package bayesian_data;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class training_data implements lcm.lcm.LCMEncodable
{
    public int num_training_;
    public bayesian_data.training_pair training_vertex_[];
 
    public training_data()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xd2783642ff2f952aL;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(bayesian_data.training_data.class))
            return 0L;
 
        classes.add(bayesian_data.training_data.class);
        long hash = LCM_FINGERPRINT_BASE
             + bayesian_data.training_pair._hashRecursive(classes)
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
        outs.writeInt(this.num_training_); 
 
        for (int a = 0; a < this.num_training_; a++) {
            this.training_vertex_[a]._encodeRecursive(outs); 
        }
 
    }
 
    public training_data(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public training_data(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static bayesian_data.training_data _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        bayesian_data.training_data o = new bayesian_data.training_data();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.num_training_ = ins.readInt();
 
        this.training_vertex_ = new bayesian_data.training_pair[(int) num_training_];
        for (int a = 0; a < this.num_training_; a++) {
            this.training_vertex_[a] = bayesian_data.training_pair._decodeRecursiveFactory(ins);
        }
 
    }
 
    public bayesian_data.training_data copy()
    {
        bayesian_data.training_data outobj = new bayesian_data.training_data();
        outobj.num_training_ = this.num_training_;
 
        outobj.training_vertex_ = new bayesian_data.training_pair[(int) num_training_];
        for (int a = 0; a < this.num_training_; a++) {
            outobj.training_vertex_[a] = this.training_vertex_[a].copy();
        }
 
        return outobj;
    }
 
}
