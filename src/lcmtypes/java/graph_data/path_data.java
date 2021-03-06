/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package graph_data;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class path_data implements lcm.lcm.LCMEncodable
{
    public int path_size_;
    public long cell_[];
 
    public path_data()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x2388702fbaa664a7L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(graph_data.path_data.class))
            return 0L;
 
        classes.add(graph_data.path_data.class);
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
        outs.writeInt(this.path_size_); 
 
        for (int a = 0; a < this.path_size_; a++) {
            outs.writeLong(this.cell_[a]); 
        }
 
    }
 
    public path_data(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public path_data(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static graph_data.path_data _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        graph_data.path_data o = new graph_data.path_data();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.path_size_ = ins.readInt();
 
        this.cell_ = new long[(int) path_size_];
        for (int a = 0; a < this.path_size_; a++) {
            this.cell_[a] = ins.readLong();
        }
 
    }
 
    public graph_data.path_data copy()
    {
        graph_data.path_data outobj = new graph_data.path_data();
        outobj.path_size_ = this.path_size_;
 
        outobj.cell_ = new long[(int) path_size_];
        if (this.path_size_ > 0)
            System.arraycopy(this.cell_, 0, outobj.cell_, 0, this.path_size_); 
        return outobj;
    }
 
}

