/* LCM type definition class file
 * This file was automatically generated by lcm-gen
 * DO NOT MODIFY BY HAND!!!!
 */

package graph_data;
 
import java.io.*;
import java.util.*;
import lcm.lcm.*;
 
public final class prediction_data implements lcm.lcm.LCMEncodable
{
    public boolean bayesian_opt_flag;
 
    public prediction_data()
    {
    }
 
    public static final long LCM_FINGERPRINT;
    public static final long LCM_FINGERPRINT_BASE = 0x296023e0845baad8L;
 
    static {
        LCM_FINGERPRINT = _hashRecursive(new ArrayList<Class<?>>());
    }
 
    public static long _hashRecursive(ArrayList<Class<?>> classes)
    {
        if (classes.contains(graph_data.prediction_data.class))
            return 0L;
 
        classes.add(graph_data.prediction_data.class);
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
        outs.writeByte( this.bayesian_opt_flag ? 1 : 0); 
 
    }
 
    public prediction_data(byte[] data) throws IOException
    {
        this(new LCMDataInputStream(data));
    }
 
    public prediction_data(DataInput ins) throws IOException
    {
        if (ins.readLong() != LCM_FINGERPRINT)
            throw new IOException("LCM Decode error: bad fingerprint");
 
        _decodeRecursive(ins);
    }
 
    public static graph_data.prediction_data _decodeRecursiveFactory(DataInput ins) throws IOException
    {
        graph_data.prediction_data o = new graph_data.prediction_data();
        o._decodeRecursive(ins);
        return o;
    }
 
    public void _decodeRecursive(DataInput ins) throws IOException
    {
        this.bayesian_opt_flag = ins.readByte()!=0;
 
    }
 
    public graph_data.prediction_data copy()
    {
        graph_data.prediction_data outobj = new graph_data.prediction_data();
        outobj.bayesian_opt_flag = this.bayesian_opt_flag;
 
        return outobj;
    }
 
}

