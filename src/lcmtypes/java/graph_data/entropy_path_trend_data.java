/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package graph_data;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class entropy_path_trend_data implements lcm.lcm.LCMEncodable
{
    public int agent_idx_;
    public int num_iteration_;
    public double entropy_path_[];
 
    public entropy_path_trend_data()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0xcc821e30b337ce82L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(graph_data.entropy_path_trend_data.class))
            return 0L;
 
        classes.add(graph_data.entropy_path_trend_data.class);
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
        outs.writeInt(this.agent_idx_); 
 
        outs.writeInt(this.num_iteration_); 
 
        for (int a = 0; a < this.num_iteration_; a++) {
            outs.writeDouble(this.entropy_path_[a]); 
        }
 
    }
 
    public entropy_path_trend_data(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public entropy_path_trend_data(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static graph_data.entropy_path_trend_data _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        graph_data.entropy_path_trend_data o = new graph_data.entropy_path_trend_data();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.agent_idx_ = ins.readInt();
 
        this.num_iteration_ = ins.readInt();
 
        this.entropy_path_ = new double[(int) num_iteration_];
        for (int a = 0; a < this.num_iteration_; a++) {
            this.entropy_path_[a] = ins.readDouble();
        }
 
    }
 
    public graph_data.entropy_path_trend_data copy()
    {
        graph_data.entropy_path_trend_data outobj = new graph_data.entropy_path_trend_data();
        outobj.agent_idx_ = this.agent_idx_;
 
        outobj.num_iteration_ = this.num_iteration_;
 
        outobj.entropy_path_ = new double[(int) num_iteration_];
        if (this.num_iteration_ > 0)
            System.arraycopy(this.entropy_path_, 0, outobj.entropy_path_, 0, this.num_iteration_); 
        return outobj;
    }
 
}

