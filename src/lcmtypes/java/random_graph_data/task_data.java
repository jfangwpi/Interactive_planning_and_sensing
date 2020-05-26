/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package random_graph_data;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class task_data implements lcm.lcm.LCMEncodable
{
    public long target_pos_;
    public long idx_;
 
    public task_data()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x6e0dabb31cc0c5deL;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(random_graph_data.task_data.class))
            return 0L;
 
        classes.add(random_graph_data.task_data.class);
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
        outs.writeLong(this.target_pos_); 
 
        outs.writeLong(this.idx_); 
 
    }
 
    public task_data(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public task_data(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static random_graph_data.task_data _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        random_graph_data.task_data o = new random_graph_data.task_data();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.target_pos_ = ins.readLong();
 
        this.idx_ = ins.readLong();
 
    }
 
    public random_graph_data.task_data copy()
    {
        random_graph_data.task_data outobj = new random_graph_data.task_data();
        outobj.target_pos_ = this.target_pos_;
 
        outobj.idx_ = this.idx_;
 
        return outobj;
    }
 
}

